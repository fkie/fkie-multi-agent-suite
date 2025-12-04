
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import fkie_mas_daemon as nmd
import time
from typing import Callable
from typing import Dict
from typing import List
from typing import Text
from typing import Tuple
from typing import Union

from functools import partial
import json
import os
import psutil
import threading

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Subscription
from rclpy.qos import QoSCompatibility, qos_check_compatible
from rclpy.topic_endpoint_info import TopicEndpointInfo
from composition_interfaces.srv import ListNodes
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.srv import GetAvailableTransitions
from fkie_mas_daemon.monitor_servicer import MonitorServicer

from fkie_mas_pylib.interface.runtime_interface import EndpointInfo
from fkie_mas_pylib.interface.runtime_interface import IncompatibleQos
from fkie_mas_pylib.interface.runtime_interface import RosComposable
from fkie_mas_pylib.interface.runtime_interface import LifecycleTransition
from fkie_mas_pylib.interface.runtime_interface import RosLifecycleState
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
from fkie_mas_pylib.interface import SelfEncoder

LIFECYCLE_AVAILABLE = False
try:
    from lifecycle_msgs.msg import TransitionEvent
    LIFECYCLE_AVAILABLE = True
except:
    pass


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

    def __init__(self, *,
                 cb_nodes: Callable[[List[RosNode]], None],
                 cb_topics: Callable[[Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic]], None],
                 cb_services: Callable[[Dict[Tuple[ServiceNameWoPrefix, ServiceType], RosService]], None],
                 cb_composables: Callable[[List[RosComposable]], None],
                 cb_lifecycle: Callable[[List[RosLifecycleState]], None],
                 monitor_servicer: MonitorServicer = None):
        Log.debug("Create RosStateJsonify")
        self._lifecycle_state_group = MutuallyExclusiveCallbackGroup()
        self._shutdown = False
        self._cb_nodes = cb_nodes
        self._cb_topics = cb_topics
        self._cb_services = cb_services
        self._cb_composables = cb_composables
        self._cb_lifecycle = cb_lifecycle
        self.monitor_servicer = monitor_servicer
        self._local_node_names: List[str] = []
        self._composable_nodes: Dict[NodeId, RosComposable] = {}
        self._lock = threading.RLock()
        self._lifecycle_subscriptions: Dict[NodeId, Subscription] = {}
        self._local_addresses = get_local_addresses()
        self._participant_infos: Dict[ParticipantGid, ParticipantEntitiesInfo] = {}
        self._ros_service_dict: Dict[str, RosService] = {}
        self._ros_topic_dict: Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic] = {}
        self._use_name_as_node_id = self.get_rwm_implementation() in ["rmw_zenoh_cpp"]

    def stop(self):
        self._shutdown = True

    def get_rwm_implementation(self) -> Text:
        result = "rmw_fastrtps_cpp"
        if "RMW_IMPLEMENTATION" in os.environ:
            result = os.environ["RMW_IMPLEMENTATION"]
        if not result:
            if os.environ["ROS_DISTRO"] == "jazzy":
                result = "rmw_fastrtps_cpp"
        return result

    def _on_lifecycle_event(self, msg: TransitionEvent, node_id: str, node_name: NodeFullName = "invalid"):
        if (node_name != "invalid"):
            update_thread = threading.Thread(target=self._thread_update_lifecycle_call,
                                             args=(node_id, node_name,), daemon=True)
            update_thread.start()

    def _subscribe_lifecycle(self, *, topic_name, node_id: str, node_name: NodeFullName) -> bool:
        if LIFECYCLE_AVAILABLE and node_id not in self._lifecycle_subscriptions:
            sub = nmd.ros_node.create_subscription(TransitionEvent, topic_name, partial(
                self._on_lifecycle_event, node_id=node_id, node_name=node_name), 1)
            self._lifecycle_subscriptions[node_id] = sub
            return True
        return False

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
        with self._lock:
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
    def update_state(self, forceRefresh: bool) -> None:
        Log.debug(
            f"{self.__class__.__name__}: create graph for websocket")
        if forceRefresh:
            with self._lock:
                self._composable_nodes = {}
        # clear the warnings
        cached_data: CachedData = CachedData()
        cached_data.screens = screen.get_active_screens()
        node_ids: List[NodeId] = []
        result: List[RosNode] = []
        self._local_node_names = []
        transition_event_publisher = []
        new_transition_event_nodes: List[RosNode] = []
        found_composable_nodes: List[NodeId] = []
        new_composable_nodes: List[RosNode] = []
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
                ros_node, is_new_node = self._get_node_from(
                    pub_info.node_namespace, pub_info.node_name, gid, cached_data)
                tp = self._get_topic_from(topic_name, pub_info.topic_type, cached_data)
                # topic or service ?
                if isinstance(tp, RosTopic):
                    Log.debug(
                        f"{self.__class__.__name__}:      add publisher {ros_node.id} {pub_info.node_namespace}/{pub_info.node_name} for {tp.name}")
                    if not self._has_endpoint_info(ros_node.id, tp.publisher):
                        endpoint_info = EndpointInfo(t_gid, ros_node.id, self._get_qos(pub_info.qos_profile), [])
                        tp.publisher.append(endpoint_info)
                    tid = tp.get_topic_id()
                    if not self._has_topic_id(tid, ros_node.publishers):
                        ros_node.publishers.append(tid)
                    pub_qos.append(QosPub(ros_node, pub_info.qos_profile))
                    # the node is a system node if topic type is from MAS messages
                    discover_state_publisher = 'fkie_mas_msgs::msg::dds_::DiscoveredState_' in pub_info.topic_type
                    endpoint_publisher = 'fkie_mas_msgs::msg::dds_::Endpoint_' in pub_info.topic_type
                    ros_node.system_node |= ros_node.system_node or discover_state_publisher or endpoint_publisher
                    # check for lifecycle transition event topic
                    if self._is_local_lifecycle_transition_topic(tp.name, tp.msg_type, ros_node):
                        if self._subscribe_lifecycle(topic_name=tp.name, node_id=ros_node.id, node_name=ros_node.name):
                            new_transition_event_nodes.append(ros_node)
                        transition_event_publisher.append(ros_node.id)
                else:
                    is_request = topic_name[:2] == 'rq'
                    if not is_request:
                        # it is a publisher for a Reply service.
                        # We check whenever it is a composable services
                        if self._is_local_composable_service(tp.name, tp.srv_type, ros_node):
                            with self._lock:
                                if ros_node.id not in self._composable_nodes:
                                    self._composable_nodes[ros_node.id] = RosComposable(ros_node.name, ros_node.id, [])
                                    new_composable_nodes.append(ros_node)
                                found_composable_nodes.append(ros_node.id)
                    elif is_request and ros_node.id not in tp.requester:
                        Log.debug(
                            f"{self.__class__.__name__}:      add requester {ros_node.id} {pub_info.node_namespace}/{pub_info.node_name}")
                        tp.requester.append(ros_node.id)
                if is_new_node:
                    result.append(ros_node)
                    node_ids.append(ros_node.id)

            sub_infos = nmd.ros_node.get_subscriptions_info_by_topic(topic_name, True)
            for sub_info in sub_infos:
                if '_NODE_NAME_UNKNOWN_' in sub_info.node_name or '_NODE_NAMESPACE_UNKNOWN_' in sub_info.node_namespace:
                    continue
                node_full_name = os.path.join(sub_info.node_namespace, sub_info.node_name)
                gid = node_full_name if self._use_name_as_node_id else self._guid_arr_to_str(
                    sub_info.endpoint_gid[0:12])
                t_gid = self._guid_arr_to_str(sub_info.endpoint_gid)
                ros_node, is_new_node = self._get_node_from(
                    sub_info.node_namespace, sub_info.node_name, gid, cached_data)
                try:
                    tp = self._get_topic_from(topic_name, sub_info.topic_type, cached_data)
                    # topic or service ?
                    if isinstance(tp, RosTopic):
                        Log.debug(
                            f"{self.__class__.__name__}:      add subscriber {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name} for {tp.name}")
                        # check for QoS compatibility with publisher nodes
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
                        is_request = topic_name[:2] == 'rq'
                        if is_request and ros_node.id not in tp.provider:
                            Log.debug(
                                f"{self.__class__.__name__}:      add provider {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name}")
                            tp.provider.append(ros_node.id)
                            # The node subscribes a Request service type. We add this service to the node.
                            # The node is now the provider of this service
                            ros_node.services.append(tp.get_topic_id())
                except Exception:
                    import traceback
                    Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")
                if is_new_node:
                    result.append(ros_node)
                    node_ids.append(ros_node.id)

        # get services if zenoh is enabled
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
                                node.services.append(srv.get_topic_id())
                                if self._is_local_composable_service(service_name, service_type, node):
                                    with self._lock:
                                        if node.id not in self._composable_nodes:
                                            self._composable_nodes[node.id] = RosComposable(node.name, node.id, [])
                                            new_composable_nodes.append(node)
                                        found_composable_nodes.append(node.id)
                except Exception:
                    import traceback
                    Log.debug(
                        f"{self.__class__.__name__}: ignored exception for node {node_ns}/{node_base_name}: {traceback.format_exc()}")

        # update nodes with participant infos
        with self._lock:
            for node in result:
                if node.gid in self._participant_infos:
                    participant = self._participant_infos[node.gid]
                    Log.debug(
                        f"{self.__class__.__name__}:     set unicast locators: {participant.unicast_locators} for {node.name}")
                    node.location = participant.unicast_locators
                    # check if one of locations has a local IP address
                    for loc in node.location:
                        is_local = self.is_location_local(loc)
                        if is_local:
                            if not node.is_local:
                                self._local_node_names.append(node.name)
                            node.is_local = is_local
                            break
                    Log.debug(
                        f"{self.__class__.__name__}:     set unicast locators: {participant.unicast_locators} for {node.name}")
                    node.enclave = participant.enclave
                node.gid = None

        with self._lock:
            # cleanup the transition event subscriptions
            old_transitions_nodes = set(self._lifecycle_subscriptions.keys()) - set(transition_event_publisher)
            for node_id in old_transitions_nodes:
                nmd.ros_node.destroy_subscription(self._lifecycle_subscriptions[node_id])
                del self._lifecycle_subscriptions[node_id]
            # update life cycle status using services, as messages may have been missed via topics
            for node in new_transition_event_nodes:
                update_thread = threading.Thread(target=self._thread_update_lifecycle_call,
                                                 args=(node.id, node.name), daemon=True)
                update_thread.start()

            # cleanup the composable nodes
            old_container_nodes = set(list(self._composable_nodes.keys())) - set(found_composable_nodes)
            for node_id in old_container_nodes:
                del self._composable_nodes[node_id]
            # update the composable node list
            for node in new_composable_nodes:
                update_thread = threading.Thread(target=self._thread_update_composables_call,
                                                 args=(node.id, node.name), daemon=True)
                update_thread.start()

        self._ros_service_dict = cached_data.service_objs
        self._ros_topic_dict = cached_data.topic_objs

        if self._shutdown:
            return
        if self._cb_nodes:
            self._cb_nodes(result)
        if self._cb_topics:
            self._cb_topics(cached_data.topic_objs)
        if self._cb_services:
            self._cb_services(cached_data.service_objs)

    def _get_node_from(self, node_ns: str, node_name: str, gid: ParticipantGid, data: CachedData) -> Tuple[RosNode, IsNew]:
        key = (node_ns, node_name, gid)
        if key not in data.node_dict:
            full_name = os.path.join(node_ns, node_name)
            Log.debug(f"{self.__class__.__name__}:   create node: {full_name}")
            node_id = full_name
            if full_name != gid:
                node_id = f"{full_name}-{gid}"
            ros_node = RosNode(node_id, full_name)
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
            if not ros_node.is_local:
                # check the composable nodes
                with self._lock:
                    for composable in self._composable_nodes.values():
                        if full_name in composable.nodes:
                            ros_node.is_local = True
                            self._local_node_names.append(full_name)
                            break
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

    def _is_local_composable_service(self, service_name: ServiceNameWoPrefix, service_type: ServiceType, ros_node: RosNode) -> bool:
        if not ros_node.is_local:
            return False
        if service_name.endswith('/_container/list_nodes') and service_type == "composition_interfaces/srv/ListNodes":
            return True
        return False

    # Checks whether the specified topic is a lifecycle events topic and belongs to a local node.
    # We also need to check the composable nodes.
    def _is_local_lifecycle_transition_topic(self, topic_name: TopicNameWoPrefix, topic_type: TopicType, ros_node: RosNode) -> bool:
        try:
            if not ros_node.is_local:
                found = False
                with self._lock:
                    for composable in self._composable_nodes.values():
                        if ros_node.name in composable.nodes:
                            found = True
                            break
                if not found:
                    return False
            return topic_name.endswith('/transition_event') and topic_type == "lifecycle_msgs/msg/TransitionEvent"
        except Exception:
            import traceback
            Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")

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

    # Updates the lifecycle state of the specified node.
    # It calls the services /get_state and /get_available_transitions
    def _thread_update_lifecycle_call(self, node_id: str, node_name: NodeFullName):
        error_msgs: List[str] = []
        try:
            wait_futures: List[WaitFuture] = []
            Log.debug(f"{self.__class__.__name__}:  update lifecycle state '{node_name}'")
            create_service_future(nmd.ros_node,
                                  wait_futures=wait_futures,
                                  type="lifecycle state",
                                  node_name="",
                                  service_name=f"{node_name}/get_state",
                                  srv_type=GetState,
                                  request=GetState.Request())
            Log.debug(f"{self.__class__.__name__}:  update lifecycle transitions '{node_name}'")
            create_service_future(nmd.ros_node,
                                  wait_futures=wait_futures,
                                  type="lifecycle transition",
                                  node_name="",
                                  service_name=f"{node_name}/get_available_transitions",
                                  srv_type=GetAvailableTransitions,
                                  request=GetAvailableTransitions.Request())
            # wait until all service are finished of timeouted
            wait_until_futures_done(wait_futures, 3.0)
            # handle response
            lifecycle_state = RosLifecycleState(id=node_id, name=node_name)
            for wait_future in wait_futures:
                if wait_future.finished:
                    if wait_future.type == "lifecycle state":
                        if wait_future.finished:
                            try:
                                response = wait_future.future.result()
                                if response:
                                    lifecycle_state.state = response.current_state.label
                            except Exception as exception:
                                error_msgs.append(
                                    f"{self.__class__.__name__}:-> failed to update lifecycle state of '{node_name}': '{exception}'")
                    elif wait_future.type == "lifecycle transition":
                        try:
                            response = wait_future.future.result()
                            if response:
                                for transition in response.available_transitions:
                                    lifecycle_state.available_transitions.append(
                                        LifecycleTransition(transition.transition.label, transition.transition.id))
                        except Exception as exception:
                            error_msgs.append(
                                f"{self.__class__.__name__}:-> failed to update lifecycle transitions of '{node_name}': '{exception}'")
                else:
                    error_msgs.append(
                        f"{self.__class__.__name__}:-> Timeout while update {wait_future.type} of '{node_name}'")
                wait_future.client.destroy()
            # callback
            with self._lock:
                if not self._shutdown and self._cb_lifecycle:
                    self._cb_lifecycle([lifecycle_state])
        except:
            import traceback
            error_msgs.append(traceback.format_exc())
        finally:
            if len(error_msgs) > 0:
                with self._lock:
                    warnings_group: SystemWarningGroup = SystemWarningGroup(SystemWarningGroup.ID_ROS_STATE)
                    for msg in error_msgs:
                        warnings_group.append(SystemWarning(msg=msg))
                        Log.warn(msg)
                    if self.monitor_servicer:
                        self.monitor_servicer.update_warning_groups([warnings_group])

    # Updates the list of composable nodes in the specified container node.
    def _thread_update_composables_call(self, node_id: str, node_name: NodeFullName, retry: int = 1):
        error_msgs: List[str] = []
        try:
            wait_futures: List[WaitFuture] = []
            Log.debug(f"{self.__class__.__name__}:  update composables nodes for '{node_name}'")
            create_service_future(nmd.ros_node,
                                  wait_futures=wait_futures,
                                  type="composable",
                                  node_name="",
                                  service_name=f"{node_name}/_container/list_nodes",
                                  srv_type=ListNodes,
                                  request=ListNodes.Request())
            # wait until all service are finished of timeouted
            wait_until_futures_done(wait_futures, 1.5)
            # handle response
            composable = RosComposable(container_name=node_name, node_id=node_id)
            for wait_future in wait_futures:
                finished = False
                if wait_future.finished:
                    finished = True
                    if wait_future.type == "composable":
                        if wait_future.finished:
                            try:
                                response = wait_future.future.result()
                                if response:
                                    for cn_name in response.full_node_names:
                                        composable.nodes.append(cn_name)
                                    with self._lock:
                                        self._composable_nodes[node_id] = composable
                                        if not self._shutdown and self._cb_composables:
                                            self._cb_composables([composable])
                            except Exception as exception:
                                error_msgs.append(
                                    f"{self.__class__.__name__}:-> failed to update composable nodes of '{node_name}': '{exception}'")
                wait_future.client.destroy()
                if not finished:
                    if retry < 3:
                        update_thread = threading.Thread(target=self._thread_update_composables_call,
                                                         args=(node_id, node_name, retry + 1), daemon=True)
                        update_thread.start()
                    else:
                        error_msgs.append(
                            f"{self.__class__.__name__}:-> Timeout while update {wait_future.type} of '{node_name}'")

        except:
            import traceback
            error_msgs.append(traceback.format_exc())
        finally:
            if len(error_msgs) > 0:
                with self._lock:
                    warnings_group: SystemWarningGroup = SystemWarningGroup(SystemWarningGroup.ID_ROS_STATE)
                    for msg in error_msgs:
                        warnings_group.append(SystemWarning(msg=msg))
                        Log.warn(msg)
                    if self.monitor_servicer:
                        self.monitor_servicer.update_warning_groups([warnings_group])
