
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import time
from typing import Dict
from typing import List
from typing import Text
from typing import Tuple
from typing import Union

import os
import psutil
import threading

from rclpy.qos import QoSCompatibility, qos_check_compatible
from rclpy.topic_endpoint_info import TopicEndpointInfo
from composition_interfaces.srv import ListNodes
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.srv import GetAvailableTransitions
from fkie_mas_daemon.monitor_servicer import MonitorServicer

from fkie_mas_pylib.interface.runtime_interface import EndpointInfo
from fkie_mas_pylib.interface.runtime_interface import IncompatibleQos
from fkie_mas_pylib.interface.runtime_interface import RosNode
from fkie_mas_pylib.interface.runtime_interface import RosTopic
from fkie_mas_pylib.interface.runtime_interface import RosTopicId
from fkie_mas_pylib.interface.runtime_interface import RosQos
from fkie_mas_pylib.interface.runtime_interface import RosService
from fkie_mas_pylib.interface.runtime_interface import SystemWarning
from fkie_mas_pylib.interface.runtime_interface import SystemWarningGroup
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib import names
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.system.host import get_local_addresses
from fkie_mas_pylib.service.future import WaitFuture
from fkie_mas_pylib.service.future import create_service_future
from fkie_mas_pylib.service.future import wait_until_futures_done
from fkie_mas_msgs.msg import ParticipantEntitiesInfo
from fkie_mas_msgs.msg import Participants

import fkie_mas_daemon as nmd


# first 12 uint8 values of the RMW_GID represented as {:02X}
ParticipantGid = str
# all uint8 values of the RMW_GID represented as {:02X}
EndpointGid = str
# /namespace/name-ParticipantGid
NodeId = str
NodeNamespace = str
NodeName = str
NodeFullName = str
TopicNameWoPrefix = str
TopicType = str
ServiceNameWoPrefix = str
ServiceName = str
ServiceType = str
IsNew = bool
IsService = bool
IsRequest = bool


class QosPub:
    node: RosNode
    qos_profile: any

    def __init__(self, node: RosNode, qos_profile: any):
        self.node = node
        self.qos_profile = qos_profile


class ComposedNodeInfo:
    container_name: NodeFullName
    container_id: str  # number service in service response
    unique_id_in_container: int

    def __init__(self, container_name: NodeFullName, unique_id_in_container: str, container_id: NodeId = ""):
        self.container_name = container_name
        self.container_id = container_id
        self.unique_id_in_container = unique_id_in_container


class CachedData:
    node_dict: Dict[Tuple[NodeNamespace, NodeName, ParticipantGid], RosNode]
    topic_objs: Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic]
    service_by_id: Dict[EndpointGid, RosService]
    service_objs: Dict[Tuple[ServiceNameWoPrefix, ServiceType], RosService]
    screens: Dict[str, str]

    def __init__(self):
        self.node_dict = {}
        self.topic_objs = {}
        self.service_by_id = {}
        self.service_objs = {}
        self.screens = {}


class RosStateJsonify:

    def __init__(self, monitor_servicer: MonitorServicer = None):
        Log.debug("Create RosStateJsonify")
        self.monitor_servicer = monitor_servicer
        self._current_nodes: List[RosNode] = []
        self._last_known_nodes: List[NodeId] = []
        self._local_node_names: List[str] = []
        self._composable_nodes: Dict[NodeFullName, NodeFullName] = {}
        self._local_addresses = get_local_addresses()
        self._participant_infos: Dict[ParticipantGid, ParticipantEntitiesInfo] = {}
        self._ros_service_dict: Dict[str, RosService] = {}
        self._ros_topic_dict: Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic] = {}
        self._ros_state_warnings: SystemWarningGroup = SystemWarningGroup(SystemWarningGroup.ID_ROS_STATE)
        self._updated_since_request = False
        self._timestamp_state = -1
        self._timestamp_state_srv_calls = -1
        self._lock_update_services = threading.RLock()
        self._thread_update_services = None
        self._use_name_as_node_id = self.get_rwm_implementation() in ["rmw_zenoh_cpp"]
        self._thread_update_service_args = None

    def get_rwm_implementation(self) -> Text:
        result = "rmw_fastrtps_cpp"
        if "RMW_IMPLEMENTATION" in os.environ:
            result = os.environ["RMW_IMPLEMENTATION"]
        if not result:
            if os.environ["ROS_DISTRO"] == "jazzy":
                result = "rmw_fastrtps_cpp"
        return result

    @classmethod
    def get_message_type(cls, dds_type: Text) -> Text:
        result = dds_type
        if result:
            result = result.replace('::', '/')
            result = result.replace('/dds_', '')
            last_slash_index = result.rfind('/')
            if last_slash_index != -1:
                underscore_index = result.rfind('_', last_slash_index)
                # if underscore_index != -1:
                #     result = result[:underscore_index]
            result = result.rstrip('_')
        return result

    @classmethod
    def get_service_type(cls, dds_service_type: Text) -> Text:
        result = dds_service_type
        for suffix in ['_Response_', '_Request_']:
            if result[-len(suffix):] == suffix:
                result = result[:-len(suffix)+1]
                break
        return cls.get_message_type(result)

    @classmethod
    def get_service_name(cls, dds_service_name: Text) -> Text:
        result = dds_service_name
        for suffix in ['Reply', 'Request']:
            if result[-len(suffix):] == suffix:
                result = result[:-len(suffix)]
                break
        return result

    def timestamp_state(self):
        return self._timestamp_state

    def is_updating(self):
        return self._thread_update_services != None

    def updated_since_request(self):
        return self._updated_since_request

    def findNode(self, name: str) -> List[int]:
        ns = names.namespace(name).rstrip('/')
        basename = names.basename(name)
        result: List[int] = []
        for process in psutil.process_iter():
            try:
                cmd_line = ' '.join(process.cmdline())
                if cmd_line.find(f"__node:={basename}") > -1 and (not ns or cmd_line.find(f"__ns:={ns}") > -1):
                    result.append(process.pid)
            except Exception:
                pass
        return result

    def is_location_local(self, location: str):
        if 'SHM' in location:
            return True
        if '127.0.' in location:
            return True
        for loc_addr in self._local_addresses:
            if loc_addr in location:
                return True
        return False

    def get_services(self) -> Dict[Tuple[ServiceNameWoPrefix, ServiceType], RosService]:
        return self._ros_service_dict

    def get_topics(self) -> Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic]:
        return self._ros_topic_dict

    def get_local_node_names(self) -> List[str]:
        return self._local_node_names

    def apply_participants(self, msg: Participants):
        # update the participant info (IP addresses)
        new_ros_state = {}
        for participant in msg.participants:
            guid = self._guid_to_str(participant.guid)
            new_ros_state[guid] = participant
        self._participant_infos = new_ros_state

    def parse_node_name(self, node_name):
        full_node_name = node_name
        if not full_node_name.startswith('/'):
            full_node_name = '/' + full_node_name
        namespace, node_basename = full_node_name.rsplit('/', 1)
        if namespace == '':
            namespace = '/'
        return node_basename, namespace

    # Creates a list of RosNode's from discovered ROS2 list
    # The status of composable or lifecycle nodes is determined by calling services. This is done in a separate thread.
    # This is done in such a way that no changes are missed. With a newer 'ts_state_updated' the parameters are cached and the thread is started again if necessary when it is finished.
    def get_nodes_as_json(self, ts_state_updated, update_participants: bool, forceRefresh: bool = False) -> List[RosNode]:
        # do not update composable and lifecycle nodes after these requests are completed
        # service calls cause graph events in ROS
        updates_with_service_calls = self._thread_update_services == None and time.time() - \
            self._timestamp_state_srv_calls > 2.
        if not forceRefresh:
            if self._updated_since_request or (ts_state_updated <= self._timestamp_state):
                self._updated_since_request = False
                Log.debug(f"{self.__class__.__name__}: report cached state")
                with self._lock_update_services:
                    return self._current_nodes
        Log.debug(
            f"{self.__class__.__name__}: create graph for websocket; update composables and lifecycle: {updates_with_service_calls}")
        # clear the warnings
        self._ros_state_warnings = SystemWarningGroup(SystemWarningGroup.ID_ROS_STATE)
        if self.monitor_servicer:
            self.monitor_servicer.update_warning_groups([self._ros_state_warnings])
        self._timestamp_state = time.time()
        cached_data: CachedData = CachedData()
        node_ids: List[NodeId] = []
        new_nodes_detected: bool = False
        composable_services: List[str] = []
        lifecycle_state_services: List[Tuple[ServiceName, NodeFullName]] = []
        lifecycle_transition_services: List[Tuple[ServiceName, NodeFullName]] = []
        result: List[RosNode] = []
        cached_data.screens = screen.get_active_screens()
        self._local_node_names = []

        if update_participants:
            self._ros_service_dict = {}
            self._ros_topic_dict = {}

        topic_list = nmd.ros_node.get_topic_names_and_types(no_demangle=True)
        for topic_name, topic_types in topic_list:
            pub_qos: List[QosPub] = []
            pub_infos = nmd.ros_node.get_publishers_info_by_topic(topic_name, True)
            for pub_info in pub_infos:
                if '_NODE_NAME_UNKNOWN_' in pub_info.node_name or '_NODE_NAMESPACE_UNKNOWN_' in pub_info.node_namespace:
                    continue
                node_full_name = os.path.join(pub_info.node_namespace, pub_info.node_name)
                gid = node_full_name if self._use_name_as_node_id else self._guid_arr_to_str(
                    pub_info.endpoint_gid[0:12])
                t_gid = self._guid_arr_to_str(pub_info.endpoint_gid)
                ros_node, is_new = self._get_node_from(
                    pub_info.node_namespace, pub_info.node_name, gid, cached_data)
                tp = self._get_topic_from(topic_name, pub_info.topic_type, cached_data)
                is_topic = isinstance(tp, RosTopic)
                # topic or service ?
                if is_topic:
                    discover_state_publisher = False
                    endpoint_publisher = False
                    Log.debug(
                        f"{self.__class__.__name__}:      add publisher {ros_node.id} {pub_info.node_namespace}/{pub_info.node_name} for {tp.name}")
                    if not self._has_endpoint_info(ros_node.id, tp.publisher):
                        endpoint_info = EndpointInfo(t_gid, ros_node.id, self._get_qos(pub_info.qos_profile), [])
                        tp.publisher.append(endpoint_info)
                    tid = tp.get_topic_id()
                    if not self._has_topic_id(tid, ros_node.publishers):
                        ros_node.publishers.append(tid)
                    discover_state_publisher = 'fkie_mas_msgs::msg::dds_::DiscoveredState_' in pub_info.topic_type
                    endpoint_publisher = 'fkie_mas_msgs::msg::dds_::Endpoint_' in pub_info.topic_type
                    ros_node.system_node |= ros_node.system_node or discover_state_publisher or endpoint_publisher
                    pub_qos.append(QosPub(ros_node, pub_info.qos_profile))
                else:
                    if not tp.is_request and ros_node.id not in tp.provider:
                        Log.debug(
                            f"{self.__class__.__name__}:      add provider {ros_node.id} {pub_info.node_namespace}/{pub_info.node_name}")
                        tp.provider.append(ros_node.id)
                        if updates_with_service_calls:
                            if self._is_local_composable_service(tp.name, ros_node):
                                composable_services.append(tp.name)
                            if self._is_local_lifecycle_state_service(tp.name, tp.srv_type, ros_node):
                                lifecycle_state_services.append((tp.name, ros_node.name))
                            if self._is_local_lifecycle_transitions_service(tp.name, tp.srv_type, ros_node):
                                lifecycle_transition_services.append((tp.name, ros_node.name))
                    elif tp.is_request and ros_node.id not in tp.requester:
                        Log.debug(
                            f"{self.__class__.__name__}:      add requester {ros_node.id} {pub_info.node_namespace}/{pub_info.node_name}")
                        tp.requester.append(ros_node.id)
                    # this are the service caller. TODO: create a new field in RosNode
                    # if is_request:
                    #     ros_node.services.append(ros_topic_id)
                if is_new:
                    result.append(ros_node)
                    node_ids.append(ros_node.id)
                    if ros_node.id not in self._last_known_nodes:
                        new_nodes_detected = True

            sub_infos = nmd.ros_node.get_subscriptions_info_by_topic(topic_name, True)
            for sub_info in sub_infos:
                if '_NODE_NAME_UNKNOWN_' in sub_info.node_name or '_NODE_NAMESPACE_UNKNOWN_' in sub_info.node_namespace:
                    continue
                node_full_name = os.path.join(sub_info.node_namespace, sub_info.node_name)
                gid = node_full_name if self._use_name_as_node_id else self._guid_arr_to_str(
                    sub_info.endpoint_gid[0:12])
                t_gid = self._guid_arr_to_str(sub_info.endpoint_gid)
                ros_node, is_new = self._get_node_from(sub_info.node_namespace, sub_info.node_name, gid, cached_data)
                try:
                    tp = self._get_topic_from(topic_name, sub_info.topic_type, cached_data)
                    is_topic = isinstance(tp, RosTopic)
                    # topic or service ?
                    if is_topic:
                        Log.debug(
                            f"{self.__class__.__name__}:      add subscriber {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name} for {tp.name}")
                        # check for compatibility with publisher nodes
                        incompatible_qos = []
                        for qp in pub_qos:
                            compatibility, reason = qos_check_compatible(qp.qos_profile, sub_info.qos_profile)
                            if compatibility != QoSCompatibility.OK:
                                incompatible_qos.append(IncompatibleQos(
                                    qp.node.id, self._qos_compatibility2str(compatibility), reason))
                        if not self._has_endpoint_info(ros_node.id, tp.subscriber):
                            endpoint_info = EndpointInfo(t_gid, ros_node.id, self._get_qos(
                                sub_info.qos_profile), incompatible_qos)
                            tp.subscriber.append(endpoint_info)
                        tid = tp.get_topic_id()
                        if not self._has_topic_id(tid, ros_node.subscribers):
                            ros_node.subscribers.append(tid)
                    else:
                        if tp.is_request and ros_node.id not in tp.provider:
                            Log.debug(
                                f"{self.__class__.__name__}:      add provider {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name}")
                            tp.provider.append(ros_node.id)
                            if updates_with_service_calls:
                                if self._is_local_composable_service(tp.name, ros_node):
                                    composable_services.append(tp.name)
                                if self._is_local_lifecycle_state_service(tp.name, tp.srv_type, ros_node):
                                    lifecycle_state_services.append((tp.name, ros_node.name))
                                if self._is_local_lifecycle_transitions_service(tp.name, tp.srv_type, ros_node):
                                    lifecycle_transition_services.append((tp.name, ros_node.name))
                        elif not tp.is_request and ros_node.id not in tp.requester:
                            Log.debug(
                                f"{self.__class__.__name__}:      add requester {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name}")
                            tp.requester.append(ros_node.id)
                        if tp.is_request:
                            tid = tp.get_topic_id()
                            if tid not in cached_data.service_by_id:
                                ros_node.services.append(tid)
                            cached_data.service_by_id[tp.id] = tp
                except Exception:
                    import traceback
                    print(f"ignored exception: {traceback.format_exc()}")
                if is_new:
                    result.append(ros_node)
                    node_ids.append(ros_node.id)
                    if ros_node.id not in self._last_known_nodes:
                        new_nodes_detected = True
        # get services
        if self._use_name_as_node_id and hasattr(nmd.ros_node, "get_service_names_and_types"):
            for node in result:
                node_base_name, node_ns = self.parse_node_name(node.name)
                try:
                    service_names_and_types = nmd.ros_node.get_service_names_and_types_by_node(node_base_name, node_ns)
                    for service_name, service_types in service_names_and_types:
                        for service_type in service_types:
                            if (service_name, service_type) not in cached_data.service_objs:
                                srv = RosService(service_name, service_type)
                                cached_data.service_objs[(service_name, service_type)] = srv
                                srv.provider.append(node.id)
                                if updates_with_service_calls:
                                    if self._is_local_composable_service(service_name, node):
                                        composable_services.append(service_name)
                                    if self._is_local_lifecycle_state_service(service_name, service_type, node):
                                        lifecycle_state_services.append((service_name, node.name))
                                    if self._is_local_lifecycle_transitions_service(service_name, service_type, node):
                                        lifecycle_transition_services.append((service_name, node.name))
                                if srv.id not in cached_data.service_by_id:
                                    node.services.append(srv.get_topic_id())
                                cached_data.service_by_id[srv.id] = srv
                except Exception:
                    import traceback
                    print(f"ignored exception: {traceback.format_exc()}")

        # update composable nodes
        if new_nodes_detected:
            self._composable_nodes = {}

        self._last_known_nodes = node_ids

        # update nodes with participant infos
        for node in result:
            if node.gid in self._participant_infos:
                participant = self._participant_infos[node.gid]
                Log.debug(f"{self.__class__.__name__}:     set unicast locators: {participant.unicast_locators} for {node.name}")
                node.location = participant.unicast_locators
                # check if one of locations has a local IP address
                for loc in node.location:
                    is_local = self.is_location_local(loc)
                    if is_local:
                        node.is_local = is_local
                        break
                Log.debug(f"{self.__class__.__name__}:     set unicast locators: {participant.unicast_locators} for {node.name}")
                node.enclave = participant.enclave
            node.gid = None
        if self.monitor_servicer:
            self.monitor_servicer.update_warning_groups([self._ros_state_warnings])
        self._current_nodes = result
        self._ros_service_dict = cached_data.service_objs
        self._ros_topic_dict = cached_data.topic_objs

        # store arguments for next thread run to get services of the updated node list
        _thread_update_service_args = (
            composable_services, lifecycle_state_services, lifecycle_transition_services)
        if self._thread_update_services is None:
            self._thread_update_services = threading.Thread(
                target=self._thread_update_services_call, args=_thread_update_service_args, daemon=True)
            # self._thread_update_service_args = None
            self._thread_update_services.start()

        return self._current_nodes

    def _get_node_from(self, node_ns: str, node_name: str, gid: ParticipantGid, data: CachedData) -> Tuple[RosNode, IsNew]:
        key = (node_ns, node_name, gid)
        if key not in data.node_dict:
            full_name = os.path.join(node_ns, node_name)
            Log.debug(f"{self.__class__.__name__}:   create node: {full_name}")
            ros_node = RosNode(f"{full_name}-{gid}", full_name)
            ros_node.namespace = node_ns
            ros_node.gid = gid
            # Add active screens for a given node
            for session_name, screen_node_name in data.screens.items():
                if screen_node_name == ros_node.name:
                    Log.debug(f"{self.__class__.__name__}:     append screen: {session_name}")
                    ros_node.screens.append(session_name)
                    try:
                        ros_node.process_ids.append(int(session_name.split('.')[0]))
                    except Exception:
                        pass
            # try to find process of the node
            ros_node.process_ids.extend(self.findNode(full_name))
            ros_node.process_ids = list(set(ros_node.process_ids))
            ros_node.system_node = os.path.basename(full_name).startswith('_') or full_name in ['/rosout']
            ros_node.system_node |= node_ns == '/mas' or node_ns.startswith('/mas/')
            # if a screen is available, we assume it is a local node
            if len(ros_node.process_ids) > 0:
                ros_node.location = self._local_addresses
                ros_node.is_local = True
                self._local_node_names.append(full_name)

            data.node_dict[key] = ros_node
            return data.node_dict[key], True
        return data.node_dict[key], False

    def _guid_arr_to_str(self, gid: List[int]) -> str:
        return '.'.join('{:02X}'.format(c) for c in gid)

    def _get_topic_from(self, topic_name: str, topic_type: str, data: CachedData) -> Union[RosTopic, RosService]:
        result_obj: Union[RosTopic, RosService] = None
        if topic_name.startswith('rt/'):
            if (topic_name[2:], topic_type) not in data.topic_objs:
                topic_type_res = self.get_message_type(topic_type)
                Log.debug(f"{self.__class__.__name__}:   create topic {topic_name[2:]} ({topic_type_res})")
                result_obj = RosTopic(topic_name[2:], topic_type_res)
                data.topic_objs[(topic_name[2:], topic_type)] = result_obj
            else:
                result_obj = data.topic_objs[(topic_name[2:], topic_type)]
            return result_obj
        elif topic_name[:2] in ['rr', 'rq', 'rs']:
            srv_type = self.get_service_type(topic_type)
            # TODO: distinction between Reply/Request? Currently it is removed.
            srv_name = self.get_service_name(topic_name[2:])
            if (srv_name, srv_type) not in data.service_objs:
                srv_type_res = self.get_service_type(srv_type)
                Log.debug(f"{self.__class__.__name__}:   create service {srv_name} ({srv_type_res})")
                result_obj = RosService(srv_name, srv_type_res)
                result_obj.is_request = topic_name[:2] == 'rq'
                data.service_objs[(srv_name, srv_type)] = result_obj
            else:
                result_obj = data.service_objs[(srv_name, srv_type)]
            return result_obj
        # fallback if we have not DDS prefix. e.g. while using zenoh
        elif (topic_name, topic_type) not in data.topic_objs:
            topic_type_res = self.get_message_type(topic_type)
            Log.debug(f"{self.__class__.__name__}:   create topic {topic_name} ({topic_type_res})")
            result_obj = RosTopic(topic_name, topic_type_res)
            data.topic_objs[(topic_name, topic_type)] = result_obj
        else:
            result_obj = data.topic_objs[(topic_name, topic_type)]
        return result_obj

    def _has_topic_id(self, id: RosTopicId, list: List[RosTopicId]):
        for item in list:
            if id.name == item.name and id.msg_type == item.msg_type:
                return True
        return False

    def _has_endpoint_info(self, node_id: str, list: List[EndpointInfo]):
        for item in list:
            if node_id == item.node_id:
                return True
        return False

    def _is_local_composable_service(self, service_name, ros_node: RosNode) -> bool:
        if not ros_node.is_local:
            return False
        if service_name.endswith('/_container/list_nodes'):
            # check only local nodes
            ros_node.is_container = True
            return True
        return False

    def _is_local_lifecycle_state_service(self, service_name, service_type, ros_node: RosNode) -> bool:
        if not ros_node.is_local:
            return False
        if service_name.endswith('/get_state') and service_type == "lifecycle_msgs/srv/GetState":
            # check only local nodes
            ros_node.lifecycle_state = "unknown"
            for idx in range(len(self._current_nodes)):
                if self._current_nodes[idx].name == ros_node.name:
                    ros_node.lifecycle_state = self._current_nodes[idx].lifecycle_state
            return True
        return False

    def _is_local_lifecycle_transitions_service(self, service_name, service_type, ros_node: RosNode) -> bool:
        if not ros_node.is_local:
            return False
        if service_name.endswith('/get_available_transitions') and service_type == "lifecycle_msgs/srv/GetAvailableTransitions":
            # check only local nodes
            ros_node.lifecycle_available_transitions = []
            for idx in range(len(self._current_nodes)):
                if self._current_nodes[idx].name == ros_node.name:
                    ros_node.lifecycle_available_transitions = self._current_nodes[idx].lifecycle_available_transitions
            return True
        return False

    def _thread_update_services_call(self, composable_services: List[str] = [], lifecycle_state_services: List[Tuple[ServiceName, NodeFullName]] = [], lifecycle_transition_services: List[Tuple[ServiceName, NodeFullName]] = []):
        if len(composable_services) == 0 and len(lifecycle_state_services) == 0 and lifecycle_transition_services == 0:
            self._thread_update_services = None
            return
        try:
            starts = time.time()
            wait_futures: List[WaitFuture] = []
            for service_name in composable_services:
                Log.debug(f"{self.__class__.__name__}:  update composable nodes '{service_name}'")
                create_service_future(nmd.ros_node, wait_futures, "composable", "", service_name,
                                      ListNodes, ListNodes.Request())
            # wait until all service are finished of timeouted
            wait_until_futures_done(wait_futures, 1.5)
            # handle response
            for wait_future in wait_futures:
                if wait_future.type == "composable":
                    with self._lock_update_services:
                        self._update_composable_node(self._composable_nodes, wait_future)
            # apply composable nodes to the result
            with self._lock_update_services:
                for composable_name, container_name in self._composable_nodes.items():
                    for idx in range(len(self._current_nodes)):
                        if self._current_nodes[idx].name == composable_name:
                            Log.debug(
                                f"{self.__class__.__name__}:    found composable node {self._current_nodes[idx].name}")
                            self._current_nodes[idx].container_name = container_name
                            # update location of the composable node: it is the location of the container
                            for idx_container in range(len(self._current_nodes)):
                                if self._current_nodes[idx_container].name == container_name:
                                    self._current_nodes[idx].is_local = self._current_nodes[idx_container].is_local

            with self._lock_update_services:
                # update lifecycle nodes after composable nodes are updated
                # Only then will we know whether the composable node or its container is local.
                for (service_name, node_name) in lifecycle_state_services:
                    for idx in range(len(self._current_nodes)):
                        if self._current_nodes[idx].name == node_name:
                            if self._current_nodes[idx].is_local:
                                Log.debug(f"{self.__class__.__name__}:  update lifecycle state '{service_name}'")
                                create_service_future(nmd.ros_node, wait_futures, "lc_state", "", service_name,
                                                      GetState, GetState.Request())
                for (service_name, node_name) in lifecycle_transition_services:
                    for idx in range(len(self._current_nodes)):
                        if self._current_nodes[idx].name == node_name:
                            if self._current_nodes[idx].is_local:
                                Log.debug(f"{self.__class__.__name__}:  update lifecycle transitions '{service_name}'")
                                create_service_future(nmd.ros_node, wait_futures, "lc_transition", "", service_name,
                                                      GetAvailableTransitions, GetAvailableTransitions.Request())

            # wait until all service are finished of timeouted
            wait_until_futures_done(wait_futures, 3.0)
            # handle response
            for wait_future in wait_futures:
                if wait_future.type == "lc_state":
                    with self._lock_update_services:
                        self._update_lifecycle_state(self._current_nodes, wait_future)
                elif wait_future.type == "lc_transition":
                    with self._lock_update_services:
                        self._update_lifecycle_transition(self._current_nodes, wait_future)

            if time.time() - starts > 1.0:
                msg = f"{self.__class__.__name__}: ros state update took {time.time() - starts} sec"
                self._ros_state_warnings.append(SystemWarning(msg=msg))
                Log.warn(msg)

            # apply composable nodes to the result
            with self._lock_update_services:
                if self.monitor_servicer:
                    self.monitor_servicer.update_warning_groups([self._ros_state_warnings])
        except:
            import traceback
            print(traceback.format_exc())
        finally:
            self._thread_update_services = None
            self._updated_since_request = True
            self._timestamp_state_srv_calls = time.time()

    def _update_composable_node(self, composable_nodes: Dict[NodeFullName, NodeFullName], future: WaitFuture) -> None:
        container_name = future.service_name.replace('/_container/list_nodes', '')
        if future.finished:
            try:
                response = future.future.result()
                if response:
                    for cn_name in response.full_node_names:
                        composable_nodes[cn_name] = container_name
            except Exception as exception:
                msg = f"{self.__class__.__name__}:-> failed to update composable nodes of '{container_name}': '{exception}'"
                self._ros_state_warnings.append(SystemWarning(msg=msg))
                Log.warn(msg)
        else:
            msg = f"{self.__class__.__name__}:-> Timeout while update composable nodes of '{container_name}'"
            self._ros_state_warnings.append(SystemWarning(msg=msg))
            Log.warn(msg)
        future.client.destroy()

    def _update_lifecycle_state(self, nodes: List[RosNode], future: WaitFuture) -> None:
        node_name = future.service_name.replace('/get_state', '')
        if future.finished:
            try:
                response = future.future.result()
                for idx in range(len(self._current_nodes)):
                    if self._current_nodes[idx].name == node_name:
                        if response:
                            self._current_nodes[idx].lifecycle_state = response.current_state.label
                        else:
                            self._current_nodes[idx].lifecycle_state = "unknown"
            except Exception as exception:
                msg = f"{self.__class__.__name__}:-> failed to update lifecycle state of '{node_name}': '{exception}'"
                self._ros_state_warnings.append(SystemWarning(msg=msg))
                Log.warn(msg)
        else:
            msg = f"{self.__class__.__name__}:-> Timeout while update lifecycle state of '{node_name}'"
            self._ros_state_warnings.append(SystemWarning(msg=msg))
            Log.warn(msg)
        future.client.destroy()

    def _update_lifecycle_transition(self, nodes: List[RosNode], future: WaitFuture) -> None:
        node_name = future.service_name.replace('/get_available_transitions', '')
        if future.finished:
            try:
                response = future.future.result()
                for idx in range(len(self._current_nodes)):
                    if self._current_nodes[idx].name == node_name:
                        self._current_nodes[idx].lifecycle_available_transitions = []
                        if response:
                            for transition in response.available_transitions:
                                self._current_nodes[idx].lifecycle_available_transitions.append(
                                    (transition.transition.label, transition.transition.id))
            except Exception as exception:
                msg = f"{self.__class__.__name__}:-> failed to update lifecycle transitions of '{node_name}': '{exception}'"
                self._ros_state_warnings.append(SystemWarning(msg=msg))
                Log.warn(msg)
        else:
            msg = f"{self.__class__.__name__}:-> Timeout while update lifecycle transitions of '{node_name}'"
            self._ros_state_warnings.append(SystemWarning(msg=msg))
            Log.warn(msg)
        future.client.destroy()

    def _get_qos(self, tp: TopicEndpointInfo) -> RosQos:
        return RosQos(tp.durability,
                      tp.history,
                      tp.depth,
                      tp.liveliness,
                      tp.reliability,
                      tp.deadline,
                      tp.liveliness_lease_duration,
                      tp.lifespan,
                      tp.avoid_ros_namespace_conventions)

    def _qos_compatibility2str(self, qc: QoSCompatibility) -> str:
        if qc == QoSCompatibility.OK:
            return "ok"
        return "warning"

    def _guid_to_str(self, guid: List[int]) -> ParticipantGid:
        return '.'.join('{:02X}'.format(c) for c in guid.data.tolist()[0:12])
