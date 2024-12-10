
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from typing import Dict
from typing import List
from numbers import Number
from typing import Text
from typing import Union

import os
import json
import signal
import threading
import time

from composition_interfaces.srv import ListNodes
from composition_interfaces.srv import UnloadNode

from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.runtime_interface import RosProvider
from fkie_mas_pylib.interface.runtime_interface import RosNode
from fkie_mas_pylib.interface.runtime_interface import RosTopic
from fkie_mas_pylib.interface.runtime_interface import RosService
from fkie_mas_pylib.interface.runtime_interface import LoggerConfig
from fkie_mas_pylib.interface.launch_interface import LaunchContent
from fkie_mas_pylib.defines import NM_DISCOVERY_NAME
from fkie_mas_pylib.defines import NM_NAMESPACE
from fkie_mas_pylib.defines import NMD_DEFAULT_PORT
from fkie_mas_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.names import ns_join
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.system.host import get_hostname
from fkie_mas_pylib.system.url import get_port
from fkie_mas_pylib.websocket.server import WebSocketServer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_mas_msgs.msg import DiscoveredState
from fkie_mas_msgs.msg import ParticipantEntitiesInfo
from fkie_mas_msgs.msg import Endpoint

import fkie_mas_daemon as nmd


class RosStateServicer:

    def __init__(self, websocket: WebSocketServer, test_env=False):
        Log.info("Create ros_state servicer")
        self._endpoints: Dict[str, Endpoint] = {}  # uri : Endpoint
        self._ros_state: Dict[str, ParticipantEntitiesInfo] = {}
        self._ros_node_list: List[RosNode] = None
        self.topic_name_state = f"{NM_NAMESPACE}/{NM_DISCOVERY_NAME}/rosstate"
        self.topic_name_endpoint = f"{NM_NAMESPACE}/daemons"
        self.topic_state_publisher_count = 0
        self._ts_state_updated = 0
        self._ts_state_notified = 0
        self._rate_check_discovery_node = 2  # Hz
        self._thread_check_discovery_node = None
        self._on_shutdown = False
        self.websocket = websocket
        websocket.register("ros.provider.get_list", self.get_provider_list)
        websocket.register("ros.nodes.get_list", self.get_node_list)
        websocket.register("ros.nodes.get_loggers", self.get_loggers)
        websocket.register("ros.nodes.set_logger_level", self.set_logger_level)
        websocket.register("ros.nodes.stop_node", self.stop_node)
        websocket.register("ros.subscriber.stop", self.stop_subscriber)
        websocket.register("ros.provider.get_timestamp",
                           self.get_provider_timestamp)

    def start(self):
        qos_state_profile = QoSProfile(depth=100,
                                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                       history=QoSHistoryPolicy.KEEP_LAST,
                                       reliability=QoSReliabilityPolicy.RELIABLE
                                       )
        qos_endpoint_profile = QoSProfile(depth=100,
                                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                          # history=QoSHistoryPolicy.KEEP_LAST,
                                          reliability=QoSReliabilityPolicy.RELIABLE)
        Log.info(
            f"{self.__class__.__name__}: listen for discovered items on {self.topic_name_state}")
        self.sub_discovered_state = nmd.ros_node.create_subscription(
            DiscoveredState, self.topic_name_state, self._on_msg_state, qos_profile=qos_state_profile)
        Log.info(
            f"{self.__class__.__name__}: listen for endpoint items on {self.topic_name_endpoint}")
        self.sub_endpoints = nmd.ros_node.create_subscription(
            Endpoint, self.topic_name_endpoint, self._on_msg_endpoint, qos_profile=qos_endpoint_profile)
        self._lock_check = threading.RLock()
        self._thread_check_discovery_node = threading.Thread(
            target=self._check_discovery_node, daemon=True)
        self._thread_check_discovery_node.start()

    def _endpoints_to_provider(self, endpoints) -> List[RosProvider]:
        result = []
        for uri, endpoint in endpoints.items():
            origin = uri == nmd.launcher.server.uri
            provider = RosProvider(
                name=endpoint.name, host=get_hostname(endpoint.uri), port=get_port(endpoint.uri), origin=origin, hostnames=[get_hostname(endpoint.uri)])
            result.append(provider)
        return result

    def _publish_masters(self):
        result = self._endpoints_to_provider(self._endpoints)
        self.websocket.publish('ros.provider.list', result)

    def publish_discovery_state(self):
        self.websocket.publish('ros.discovery.ready', {
            'status': self.topic_state_publisher_count > 0, 'timestamp': time.time() * 1000})

    def get_publisher_count(self):
        if hasattr(self, 'topic_name_endpoint') and self.topic_name_endpoint is not None:
            return nmd.ros_node.count_publishers(self.topic_name_endpoint)
        return -1

    def _check_discovery_node(self):
        while not self._on_shutdown:
            if self.topic_state_publisher_count:
                # check if we have a discovery node
                if nmd.ros_node.count_publishers(self.topic_name_state) == 0:
                    self.topic_state_publisher_count = 0
                    self.publish_discovery_state()
                    with self._lock_check:
                        self._ts_state_updated = time.time()
            # if a change was detected by discovery node we received _on_msg_state()
            # therefor the self._ts_state_updated was updated
            with self._lock_check:
                if self._ts_state_updated > self._ts_state_notified:
                    if time.time() - self._ts_state_notified > self._rate_check_discovery_node:
                        print(f"notify {time.time()}")
                        self._ts_state_notified = self._ts_state_updated
                        self.websocket.publish('ros.nodes.changed', {
                            "timestamp": self._ts_state_updated})
                        nmd.launcher.server.screen_servicer.system_change()
            time.sleep(1.0 / self._rate_check_discovery_node)

    def stop(self):
        '''
        Unregister the subscribed topic
        '''
        self._on_shutdown = True
        if hasattr(self, 'sub_discovered_state') and self.sub_discovered_state is not None:
            nmd.ros_node.destroy_subscription(self.sub_discovered_state)
            del self.sub_discovered_state
        if hasattr(self, 'sub_endpoints') and self.sub_endpoints is not None:
            nmd.ros_node.destroy_subscription(self.sub_endpoints)
            del self.sub_endpoints
        self.topic_state_publisher_count = 0
        self.publish_discovery_state()

    def _on_msg_state(self, msg: DiscoveredState):
        '''
        The method to handle the received Log messages.
        :param msg: the received message
        :type msg: fkie_mas_msgs.DiscoveredState<XXX>
        '''
        if not self.topic_state_publisher_count:
            self.topic_state_publisher_count = nmd.ros_node.count_publishers(
                self.topic_name_state)
            self.publish_discovery_state()
        # update the participant info (IP addresses)
        new_ros_state = {}
        for participant in msg.participants:
            guid = self._guid_to_str(participant.guid)
            new_ros_state[guid] = participant
        self._ros_state = new_ros_state
        self._ros_node_list = None
        # notify WebSocket clients, but not to often
        # notifications are sent from _check_discovery_node()
        with self._lock_check:
            self._ts_state_updated = time.time()
            print(f"update added {self._ts_state_updated}")

    def _on_msg_endpoint(self, msg: Endpoint):
        '''
        The method to handle the received Log messages.
        :param msg: the received message
        :type msg: fkie_mas_msgs.Endpoint<XXX>
        '''
        Log.info(
            f"{self.__class__.__name__}: new message on {self.topic_name_endpoint}")
        is_new = False
        if msg.on_shutdown:
            if msg.uri in self._endpoints:
                is_new = True
                del self._endpoints[msg.uri]
        elif msg.uri in self._endpoints:
            other = self._endpoints[msg.uri]
            is_new = msg.name != other.name
            is_new |= msg.ros_name != other.ros_name
            is_new |= msg.ros_domain_id != other.ros_domain_id
            is_new |= msg.pid != other.pid
        else:
            is_new = True
        if is_new:
            self._endpoints[msg.uri] = msg
            self._publish_masters()

    def get_provider_list(self) -> str:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.provider.get_list]")
        return json.dumps(self._endpoints_to_provider(self._endpoints), cls=SelfEncoder)

    def get_node_list(self, forceRefresh: bool = True) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.nodes.get_list]")
        node_list: List[RosNode] = self._get_ros_node_list(forceRefresh)

        return json.dumps(node_list, cls=SelfEncoder)

    def get_loggers(self, name: str) -> str:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.nodes.get_loggers] for '{name}', not implemented")
        loggerConfigs: List[LoggerConfig] = []
        return json.dumps({'result': False, 'logger': loggerConfigs, 'message': 'not implemented'}, cls=SelfEncoder)

    def set_logger_level(self, name: str, logger: List[LoggerConfig]) -> str:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.nodes.set_logger_level] for '{name}', not implemented")
        return json.dumps({'result': False, 'message': 'not implemented'}, cls=SelfEncoder)

    def stop_node(self, name: str) -> bool:
        Log.info(f"{self.__class__.__name__}: Request to stop node '{name}'")
        node: RosNode = self.get_ros_node(name)
        if node is None:
            node = self.get_ros_node_by_id(name)
        unloaded = False
        result = json.dumps(
            {'result': False, 'message': f'{name} not found'}, cls=SelfEncoder)
        if node is not None:
            if node.parent_id:
                unloaded = self.stop_composed_node(node)
            if not unloaded:
                result = nmd.launcher.server.screen_servicer.kill_node(
                    os.path.join(node.namespace, node.name), signal.SIGTERM)
        if unloaded:
            result = json.dumps(
                {'result': True, 'message': ''}, cls=SelfEncoder)
        nmd.launcher.server.screen_servicer.system_change()
        if node is not None:
            nmd.launcher.server.launch_servicer.node_stopped(
                os.path.join(node.namespace, node.name))
        return result

    def stop_subscriber(self, topic_name: str) -> bool:
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.subscriber.stop]: {str(topic_name)}")
        ns, name = ros2_subscriber_nodename_tuple(topic_name)
        return self.stop_node(os.path.join(ns, name))

    def get_provider_timestamp(self, timestamp) -> str:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.provider.get_timestamp], timestamp: {timestamp}")
        # Log.info("getProviderList: {0}".format(json.dumps(self.provider_list, cls=SelfEncoder)))
        return json.dumps({'timestamp': time.time() * 1000, "diff": time.time() * 1000 - float(timestamp)}, cls=SelfEncoder)

    def stop_composed_node(self, node: RosNode) -> bool:
        # try to unload node from container
        node_name = ns_join(node.namespace, node.name)
        Log.info(
            f"{self.__class__.__name__}: -> unload '{node_name}' from '{node.parent_id}'")
        container_node: RosNode = self.get_ros_node_by_id(node.parent_id)
        if container_node is not None:
            container_name = ns_join(
                container_node.namespace, container_node.name)
            try:
                node_unique_id = self.get_composed_node_id(
                    container_name, node_name)
            except Exception as err:
                print(f"{self.__class__.__name__}: unload ERR {err}")
                # return json.dumps({'result': False, 'message': str(err)}, cls=SelfEncoder)
                return False

            service_unload_node = f'{container_name}/_container/unload_node'
            Log.info(
                f"{self.__class__.__name__}:-> unload '{node_name}' with id '{node_unique_id}' from '{service_unload_node}'")

            request = UnloadNode.Request()
            request.unique_id = node_unique_id
            response = nmd.launcher.call_service(
                service_unload_node, UnloadNode, request)
            if not response.success:
                Log.warn(
                    f"{self.__class__.__name__}:-> unload '{node_name}' error '{response.error_message}'")
                return False
            return True
        else:
            Log.warn(
                f"{self.__class__.__name__}:-> {node.parent_id} not found!")
            return False

    def get_composed_node_id(self, container_name: str, node_name: str) -> Number:
        service_list_nodes = f'{container_name}/_container/list_nodes'
        Log.debug(
            f"{self.__class__.__name__}: list nodes from '{service_list_nodes}'")
        request_list = ListNodes.Request()
        response_list = nmd.launcher.call_service(
            service_list_nodes, ListNodes, request_list)
        for name, unique_id in zip(response_list.full_node_names, response_list.unique_ids):
            if name == node_name:
                return unique_id
        return -1

    def _get_ros_node_list(self, forceRefresh: bool = False) -> List[RosNode]:
        # self._ros_node_list is cleared on updates in _on_msg_state()
        # otherwise we use a cached list
        if self._ros_node_list is None or forceRefresh:
            self._ros_node_list = self.node_list_to_json()
        return self._ros_node_list

    def get_ros_node(self, node_name: str) -> Union[RosNode, None]:
        node_list: List[RosNode] = self._get_ros_node_list()
        for node in node_list:
            if node_name == ns_join(node.namespace, node.name):
                return node
        return None

    def get_ros_node_by_id(self, node_id: str) -> Union[RosNode, None]:
        node_list: List[RosNode] = self._get_ros_node_list()
        for node in node_list:
            if node_id == node.id:
                return node
        return None

    def _guid_to_str(self, guid):
        return '.'.join('{:02X}'.format(c) for c in guid.data.tolist()[0:12])

    def _guid_arr_to_str(self, guid):
        return '.'.join('{:02X}'.format(c) for c in guid)

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

    def node_list_to_json(self) -> List[RosNode]:
        node_dict = {}
        topic_by_id = {}
        topic_objs = {}
        service_by_id = {}
        service_objs = {}
        Log.debug(f"{self.__class__.__name__}: create graph for websocket")

        def _get_node_from(node_ns, node_name, node_guid):
            key = (node_ns, node_name, node_guid)
            if key not in node_dict:
                full_name = os.path.join(node_ns, node_name)
                Log.debug(
                    f"{self.__class__.__name__}:   create node: {full_name}")
                ros_node = RosNode(f"{full_name}-{node_guid}", full_name)
                # search node with same guid, we assume it is the manager
                for (_ns, _name, _guid), _node in node_dict.items():
                    if node_guid == _guid and _node.parent_id is None:
                        ros_node.parent_id = _node.id

                node_dict[key] = ros_node
                if node_guid in self._ros_state:
                    participant = self._ros_state[node_guid]
                    ros_node.location = participant.unicast_locators
                    Log.debug(
                        f"{self.__class__.__name__}:     set unicast locators: {participant.unicast_locators}")
                    ros_node.namespace = node_ns
                    ros_node.enclave = participant.enclave
                # Add active screens for a given node
                screens = screen.get_active_screens(full_name)
                for session_name, _ in screens.items():
                    Log.debug(
                        f"{self.__class__.__name__}:     append screen: {session_name}")
                    ros_node.screens.append(session_name)
                ros_node.system_node = os.path.basename(
                    full_name).startswith('_') or full_name in ['/rosout']
                ros_node.system_node |= node_ns == '/mas' or node_ns.startswith('/mas/')

                return node_dict[key], True
            return node_dict[key], False

        def _get_topic_from(topic_name, topic_type):
            t_guid = self._guid_arr_to_str(pub_info.endpoint_gid)
            if topic_name.startswith('rt/'):
                if (topic_name[2:], topic_type) not in topic_objs:
                    topic_type_res = self.get_message_type(topic_type)
                    Log.debug(
                        f"{self.__class__.__name__}:   create topic {topic_name[2:]} ({topic_type_res})")
                    tp = RosTopic(topic_name[2:], topic_type_res)
                    topic_objs[(topic_name[2:], topic_type)] = tp
                    topic_by_id[t_guid] = tp
                else:
                    topic_by_id[t_guid] = topic_objs[(
                        topic_name[2:], topic_type)]
                return topic_by_id[t_guid], True, False
            elif topic_name[:2] in ['rr', 'rq', 'rs']:
                srv_type = self.get_service_type(topic_type)
                # TODO: distinction between Reply/Request? Currently it is removed.
                srv_name = self.get_service_name(topic_name[2:])
                if (srv_name, srv_type) not in service_objs:
                    srv_type_res = self.get_service_type(srv_type)
                    Log.debug(
                        f"{self.__class__.__name__}:   create service {srv_name} ({srv_type_res})")
                    srv = RosService(srv_name, srv_type_res)
                    service_objs[(srv_name, srv_type)] = srv
                    service_by_id[t_guid] = srv
                else:
                    service_by_id[t_guid] = service_objs[(
                        srv_name, srv_type)]
                return service_by_id[t_guid], False, topic_name[:2] == 'rq'
        result = []

        topic_list = nmd.ros_node.get_topic_names_and_types(True)
        for topic_name, topic_types in topic_list:
            pub_infos = nmd.ros_node.get_publishers_info_by_topic(
                topic_name, True)
            if pub_infos:
                for pub_info in pub_infos:
                    if '_NODE_NAME_UNKNOWN_' in pub_info.node_name or '_NODE_NAMESPACE_UNKNOWN_' in pub_info.node_namespace:
                        continue
                    n_guid = self._guid_arr_to_str(pub_info.endpoint_gid[0:12])
                    ros_node, is_new = _get_node_from(
                        pub_info.node_namespace, pub_info.node_name, n_guid)
                    for topic_type in topic_types:
                        tp, is_topic, is_request = _get_topic_from(
                            topic_name, topic_type)
                        # add tp.qos_profile
                        if is_topic:
                            discover_state_publisher = False
                            endpoint_publisher = False
                            Log.debug(
                                f"{self.__class__.__name__}:      add publisher {ros_node.id} {pub_info.node_namespace}/{pub_info.node_name}")
                            tp.publisher.append(ros_node.id)
                            ros_node.publishers.append(tp)
                            discover_state_publisher = 'fkie_mas_msgs::msg::dds_::DiscoveredState_' in topic_type
                            endpoint_publisher = 'fkie_mas_msgs::msg::dds_::Endpoint_' in topic_type
                            ros_node.system_node |= ros_node.system_node or discover_state_publisher or endpoint_publisher
                        else:
                            if not is_request and ros_node.id not in tp.provider:
                                Log.debug(
                                    f"{self.__class__.__name__}:      add provider {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name}")
                                tp.provider.append(ros_node.id)
                            elif is_request and ros_node.id not in tp.requester:
                                Log.debug(
                                    f"{self.__class__.__name__}:      add requester {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name}")
                                tp.requester.append(ros_node.id)
                            if not is_request:
                                ros_node.services.append(tp)

                    if is_new:
                        result.append(ros_node)
            sub_infos = nmd.ros_node.get_subscriptions_info_by_topic(
                topic_name, True)

            if sub_infos:
                for sub_info in sub_infos:
                    if '_NODE_NAME_UNKNOWN_' in sub_info.node_name or '_NODE_NAMESPACE_UNKNOWN_' in sub_info.node_namespace:
                        continue
                    n_guid = self._guid_arr_to_str(sub_info.endpoint_gid[0:12])
                    ros_node, is_new = _get_node_from(
                        sub_info.node_namespace, sub_info.node_name, n_guid)
                    for topic_type in topic_types:
                        try:
                            tp, is_topic, is_request = _get_topic_from(
                                topic_name, topic_type)
                            # add tp.qos_profile
                            if is_topic:
                                Log.debug(
                                    f"{self.__class__.__name__}:      add subscriber {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name}")
                                tp.subscriber.append(ros_node.id)
                                ros_node.subscribers.append(tp)
                            else:
                                if is_request and ros_node.id not in tp.provider:
                                    Log.debug(
                                        f"{self.__class__.__name__}:      add provider {ros_node.id} {sub_info.node_namespace}/{sub_info.node_name}")
                                    tp.provider.append(ros_node.id)
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
        return result

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
