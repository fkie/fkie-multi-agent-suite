
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from typing import Dict
from typing import List
from typing import Text
from typing import Tuple
from typing import Union

import os

from rclpy.qos import QoSCompatibility, qos_check_compatible
from rclpy.topic_endpoint_info import TopicEndpointInfo
from composition_interfaces.srv import ListNodes
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.srv import GetAvailableTransitions

from fkie_mas_pylib.interface.runtime_interface import EndpointInfo
from fkie_mas_pylib.interface.runtime_interface import IncompatibleQos
from fkie_mas_pylib.interface.runtime_interface import RosNode
from fkie_mas_pylib.interface.runtime_interface import RosTopic
from fkie_mas_pylib.interface.runtime_interface import RosQos
from fkie_mas_pylib.interface.runtime_interface import RosService
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.system.host import get_local_addresses
from fkie_mas_msgs.msg import ParticipantEntitiesInfo

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
    participant_infos: Dict[ParticipantGid, ParticipantEntitiesInfo]
    node_dict: Dict[Tuple[NodeNamespace, NodeName, ParticipantGid], RosNode]
    topic_by_id: Dict[EndpointGid, RosTopic]
    topic_objs: Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic]
    service_by_id: Dict[EndpointGid, RosService]
    service_objs: Dict[Tuple[ServiceNameWoPrefix, ServiceType], RosService]

    def __init__(self, participant_infos=Dict[ParticipantGid, ParticipantEntitiesInfo]):
        self.participant_infos = participant_infos
        self.node_dict = {}
        self.topic_by_id = {}
        self.topic_objs = {}
        self.service_by_id = {}
        self.service_objs = {}


class RosStateJsonify:

    def __init__(self):
        Log.debug("Create RosStateJsonify")
        self._last_known_nodes: List[NodeId] = []
        self._composable_nodes: Dict[NodeFullName, NodeFullName] = {}
        self._local_addresses = get_local_addresses()

    @classmethod
    def get_message_type(cls, dds_type: Text) -> Text:
        result = dds_type
        if result:
            result = result.replace('::', '/')
            result = result.replace('/dds_', '')
            # result = result.replace('/msg/dds_', '')
            # result = result.replace('/srv/dds_', '')
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
        return cls.get_message_type(result)

    def is_location_local(self, location: str):
        if 'SHM' in location:
            return True
        for loc_addr in self._local_addresses:
            if loc_addr in location:
                return True
        return False

    def get_nodes_as_json(self, participant_infos=Dict[ParticipantGid, ParticipantEntitiesInfo]) -> List[RosNode]:
        Log.debug(f"{self.__class__.__name__}: create graph for websocket")
        cached_data: CachedData = CachedData(participant_infos)
        node_ids: List[NodeId] = []
        new_nodes_detected: bool = False
        composable_services: List[str] = []
        lifecycle_state_services: List[str] = []
        lifecycle_transition_services: List[str] = []
        result: List[RosNode] = []

        topic_list = nmd.ros_node.get_topic_names_and_types(True)
        for topic_name, topic_types in topic_list:
            pub_qos: List[QosPub] = []
            pub_infos = nmd.ros_node.get_publishers_info_by_topic(topic_name, True)
            for pub_info in pub_infos:
                if '_NODE_NAME_UNKNOWN_' in pub_info.node_name or '_NODE_NAMESPACE_UNKNOWN_' in pub_info.node_namespace:
                    continue
                gid = self._guid_arr_to_str(pub_info.endpoint_gid[0:12])
                t_gid = self._guid_arr_to_str(pub_info.endpoint_gid)
                ros_node, is_new = self._get_node_from(
                    pub_info.node_namespace, pub_info.node_name, gid, cached_data)
                tp, is_topic, is_request = self._get_topic_from(topic_name, pub_info.topic_type, t_gid, cached_data)
                # topic or service ?
                if is_topic:
                    discover_state_publisher = False
                    endpoint_publisher = False
                    Log.debug(
                        f"{self.__class__.__name__}:      add publisher {ros_node.id} {pub_info.node_namespace}/{pub_info.node_name} for {tp.name}")
                    endpoint_info = EndpointInfo(ros_node.id, self._get_qos(pub_info.qos_profile), [])
                    tp.publisher.append(endpoint_info)
                    ros_node.publishers.append(tp)
                    discover_state_publisher = 'fkie_mas_msgs::msg::dds_::DiscoveredState_' in pub_info.topic_type
                    endpoint_publisher = 'fkie_mas_msgs::msg::dds_::Endpoint_' in pub_info.topic_type
                    ros_node.system_node |= ros_node.system_node or discover_state_publisher or endpoint_publisher
                    pub_qos.append(QosPub(ros_node, pub_info.qos_profile))
                else:
                    if not is_request and ros_node.id not in tp.provider:
                        Log.debug(
                            f"{self.__class__.__name__}:      add provider {ros_node.id} {pub_info.node_namespace}/{pub_info.node_name}")
                        tp.provider.append(ros_node.id)
                        if self._is_local_composable_service(tp.name, ros_node):
                            composable_services.append(tp.name)
                        if self._is_local_lifecycle_state_service(tp.name, tp.srv_type, ros_node):
                            lifecycle_state_services.append(tp.name)
                        if self._is_local_lifecycle_transitions_service(tp.name, tp.srv_type, ros_node):
                            lifecycle_transition_services.append(tp.name)
                    elif is_request and ros_node.id not in tp.requester:
                        Log.debug(
                            f"{self.__class__.__name__}:      add requester {ros_node.id} {pub_info.node_namespace}/{pub_info.node_name}")
                        tp.requester.append(ros_node.id)
                    if not is_request:
                        ros_node.services.append(tp)
                if is_new:
                    result.append(ros_node)
                    node_ids.append(ros_node.id)
                    if ros_node.id not in self._last_known_nodes:
                        new_nodes_detected = True

            sub_infos = nmd.ros_node.get_subscriptions_info_by_topic(topic_name, True)
            for sub_info in sub_infos:
                if '_NODE_NAME_UNKNOWN_' in sub_info.node_name or '_NODE_NAMESPACE_UNKNOWN_' in sub_info.node_namespace:
                    continue
                gid = self._guid_arr_to_str(sub_info.endpoint_gid[0:12])
                t_gid = self._guid_arr_to_str(pub_info.endpoint_gid)
                ros_node, is_new = self._get_node_from(sub_info.node_namespace, sub_info.node_name, gid, cached_data)
                try:
                    tp, is_topic, is_request = self._get_topic_from(topic_name, sub_info.topic_type, t_gid, cached_data)
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
                        endpoint_info = EndpointInfo(ros_node.id, self._get_qos(sub_info.qos_profile), incompatible_qos)
                        tp.subscriber.append(endpoint_info)
                        ros_node.subscribers.append(tp)
                    else:
                        if is_request and ros_node.id not in tp.provider:
                            Log.debug(
                                f"{self.__class__.__name__}:      add provider {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name}")
                            tp.provider.append(ros_node.id)
                            if self._is_local_composable_service(tp.name, ros_node):
                                composable_services.append(tp.name)
                            if self._is_local_lifecycle_state_service(tp.name, tp.srv_type, ros_node):
                                lifecycle_state_services.append(tp.name)
                            if self._is_local_lifecycle_transitions_service(tp.name, tp.srv_type, ros_node):
                                lifecycle_transition_services.append(tp.name)
                        elif not is_request and ros_node.id not in tp.requester:
                            Log.debug(
                                f"{self.__class__.__name__}:      add requester {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name}")
                            tp.requester.append(ros_node.id)
                        if is_request:
                            ros_node.services.append(tp)
                except Exception as err:
                    print(err)
                if is_new:
                    result.append(ros_node)
                    node_ids.append(ros_node.id)
                    if ros_node.id not in self._last_known_nodes:
                        new_nodes_detected = True
        # update composable nodes
        if new_nodes_detected:
            self._composable_nodes = self._update_composable_nodes(composable_services, result)
        # update lifecycle states
        self._update_lifecycle_states(lifecycle_state_services, result)
        self._update_lifecycle_transitions(lifecycle_transition_services, result)
        # apply composable nodes to the result
        for composable_name, container_name in self._composable_nodes.items():
            for idx in range(len(result)):
                if result[idx].name == composable_name:
                    Log.debug(f"{self.__class__.__name__}:    found composable node {result[idx].name}")
                    result[idx].container_name = container_name
        self._last_known_nodes = node_ids
        return result

    def _get_node_from(self, node_ns: str, node_name: str, gid: ParticipantGid, data: CachedData) -> Tuple[RosNode, IsNew]:
        key = (node_ns, node_name, gid)
        if key not in data.node_dict:
            full_name = os.path.join(node_ns, node_name)
            Log.debug(f"{self.__class__.__name__}:   create node: {full_name}")
            ros_node = RosNode(f"{full_name}-{gid}", full_name)
            if gid in data.participant_infos:
                participant = data.participant_infos[gid]
                ros_node.location = participant.unicast_locators
                ros_node.is_local = False
                # check if one of locations has a local IP address
                for loc in ros_node.location:
                    ros_node.is_local = self.is_location_local(loc)
                    if ros_node.is_local:
                        break
                Log.debug(f"{self.__class__.__name__}:     set unicast locators: {participant.unicast_locators}")
                ros_node.namespace = node_ns
                ros_node.enclave = participant.enclave
            # Add active screens for a given node
            screens = screen.get_active_screens(full_name)
            for session_name, _ in screens.items():
                Log.debug(f"{self.__class__.__name__}:     append screen: {session_name}")
                ros_node.screens.append(session_name)
            ros_node.system_node = os.path.basename(
                full_name).startswith('_') or full_name in ['/rosout']
            ros_node.system_node |= node_ns == '/mas' or node_ns.startswith('/mas/')

            data.node_dict[key] = ros_node
            return data.node_dict[key], True
        return data.node_dict[key], False

    def _guid_arr_to_str(self, gid: List[int]) -> str:
        return '.'.join('{:02X}'.format(c) for c in gid)

    def _get_topic_from(self, topic_name: str, topic_type: str, gid: EndpointGid, data: CachedData) -> Tuple[Union[RosNode, RosService], IsService, IsRequest]:
        if topic_name.startswith('rt/'):
            if (topic_name[2:], topic_type) not in data.topic_objs:
                topic_type_res = self.get_message_type(topic_type)
                Log.debug(f"{self.__class__.__name__}:   create topic {topic_name[2:]} ({topic_type_res})")
                tp = RosTopic(topic_name[2:], topic_type_res)
                tp.id = gid
                data.topic_objs[(topic_name[2:], topic_type)] = tp
                data.topic_by_id[gid] = tp
            else:
                data.topic_by_id[gid] = data.topic_objs[(topic_name[2:], topic_type)]
            return data.topic_by_id[gid], True, False
        elif topic_name[:2] in ['rr', 'rq', 'rs']:
            srv_type = self.get_service_type(topic_type)
            # TODO: distinction between Reply/Request? Currently it is removed.
            srv_name = self.get_service_name(topic_name[2:])
            if (srv_name, srv_type) not in data.service_objs:
                srv_type_res = self.get_service_type(srv_type)
                Log.debug(f"{self.__class__.__name__}:   create service {srv_name} ({srv_type_res})")
                srv = RosService(srv_name, srv_type_res)
                data.service_objs[(srv_name, srv_type)] = srv
                data.service_by_id[gid] = srv
            else:
                data.service_by_id[gid] = data.service_objs[(srv_name, srv_type)]
            return data.service_by_id[gid], False, topic_name[:2] == 'rq'

    def _is_local_composable_service(self, service_name, ros_node: RosNode) -> bool:
        if service_name.endswith('/_container/list_nodes') and ros_node.is_local:
            # check only local nodes
            ros_node.is_container = True
            return True
        return False

    def _is_local_lifecycle_state_service(self, service_name, service_type, ros_node: RosNode) -> bool:
        if ros_node.is_local and service_name.endswith('/get_state') and service_type == "lifecycle_msgs/srv/GetState":
            # check only local nodes
            ros_node.lifecycle_state = "unknown"
            return True
        return False

    def _is_local_lifecycle_transitions_service(self, service_name, service_type, ros_node: RosNode) -> bool:
        if ros_node.is_local and service_name.endswith('/get_available_transitions') and service_type == "lifecycle_msgs/srv/GetAvailableTransitions":
            # check only local nodes
            ros_node.lifecycle_available_transitions = []
            return True
        return False

    def _update_composable_nodes(self, composable_services: List[str], nodes: List[RosNode]) -> Dict[NodeFullName, NodeFullName]:
        composable_nodes: Dict[NodeFullName, NodeFullName] = {}
        for service_name in composable_services:
            Log.info(f"{self.__class__.__name__}:  update composable nodes {service_name}")
            # get composed nodes
            request = ListNodes.Request()
            response = nmd.launcher.call_service(service_name, ListNodes, request)
            container_name = service_name.replace('/_container/list_nodes', '')
            if response:
                # for cn_name, cn_unique_id in zip(response.full_node_names, response.unique_ids):
                for cn_name in response.full_node_names:
                    composable_nodes[cn_name] = container_name
            elif response is not None:
                Log.warn(
                    f"{self.__class__.__name__}:-> failed to update composable nodes of '{container_name}': '{response.error_message}'")
            else:
                Log.warn(
                    f"{self.__class__.__name__}:-> failed to update composable nodes of '{container_name}': '{response}'")
        return composable_nodes

    def _update_lifecycle_states(self, lifecycle_state_services: List[str], nodes: List[RosNode]) -> None:
        for service_name in lifecycle_state_services:
            try:
                Log.info(f"{self.__class__.__name__}:  update lifecycle state {service_name}")
                # get composed nodes
                request = GetState.Request()
                response = nmd.launcher.call_service(service_name, GetState, request)
                node_name = service_name.replace('/get_state', '')
                if response:
                    for idx in range(len(nodes)):
                        if nodes[idx].name == node_name:
                            nodes[idx].lifecycle_state = response.current_state.label
                elif response is not None:
                    Log.warn(
                        f"{self.__class__.__name__}:-> failed to update lifecycle state of '{node_name}': '{response.error_message}'")
                else:
                    Log.warn(
                        f"{self.__class__.__name__}:-> failed to update lifecycle state of '{node_name}': '{response}'")
            except:
                import traceback
                print(traceback.format_exc())

    def _update_lifecycle_transitions(self, lifecycle_transition_services: List[str], nodes: List[RosNode]) -> None:
        for service_name in lifecycle_transition_services:
            try:
                Log.info(f"{self.__class__.__name__}:  update lifecycle transitions {service_name}")
                # get composed nodes
                request = GetAvailableTransitions.Request()
                response = nmd.launcher.call_service(service_name, GetAvailableTransitions, request)
                node_name = service_name.replace('/get_available_transitions', '')
                if response:
                    for idx in range(len(nodes)):
                        if nodes[idx].name == node_name:
                            nodes[idx].lifecycle_available_transitions = []
                            for transition in response.available_transitions:
                                nodes[idx].lifecycle_available_transitions.append(
                                    (transition.transition.label, transition.transition.id))
                elif response is not None:
                    Log.warn(
                        f"{self.__class__.__name__}:-> failed to update lifecycle transitions of '{node_name}': '{response.error_message}'")
                else:
                    Log.warn(
                        f"{self.__class__.__name__}:-> failed to update lifecycle transitions of '{node_name}': '{response}'")
            except:
                import traceback
                print(traceback.format_exc())

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
        elif qc == QoSCompatibility.Warning:
            return "warning"
        return "error"
