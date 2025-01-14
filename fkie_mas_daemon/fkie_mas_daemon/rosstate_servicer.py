
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
from fkie_mas_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system.host import get_hostname
from fkie_mas_pylib.system.url import get_port
from fkie_mas_pylib.websocket.server import WebSocketServer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_mas_msgs.msg import ChangedState
from fkie_mas_msgs.msg import ParticipantEntitiesInfo
from fkie_mas_msgs.msg import Endpoint

import fkie_mas_daemon as nmd
from fkie_mas_daemon.rosstate_jsonify import ParticipantGid
from fkie_mas_daemon.rosstate_jsonify import RosStateJsonify


class RosStateServicer:

    def __init__(self, websocket: WebSocketServer, test_env=False):
        Log.info("Create ros_state servicer")
        self._endpoints: Dict[str, Endpoint] = {}  # uri : Endpoint
        self._ros_node_list: List[RosNode] = None
        self._ros_node_list_mutex = threading.RLock()
        self.service_name_get_p = f"{NM_NAMESPACE}/{NM_DISCOVERY_NAME}/get_participants"
        self.topic_name_state = f"{NM_NAMESPACE}/{NM_DISCOVERY_NAME}/changed"
        self.topic_name_endpoint = f"{NM_NAMESPACE}/daemons"
        self.topic_state_publisher_count = 0
        self._update_participants = True
        self._force_refresh = False
        self._ts_state_updated = 0
        self._ts_state_notified = 0
        self._rate_check_discovery_node = 2  # Hz
        self._thread_check_discovery_node = None
        self._on_shutdown = False
        self._state_jsonify = RosStateJsonify(self.service_name_get_p)
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
        qos_state_profile = QoSProfile(depth=10,
                                       #    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                       #    history=QoSHistoryPolicy.KEEP_LAST,
                                       #    reliability=QoSReliabilityPolicy.RELIABLE
                                       )
        qos_endpoint_profile = QoSProfile(depth=100,
                                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                          # history=QoSHistoryPolicy.KEEP_LAST,
                                          reliability=QoSReliabilityPolicy.RELIABLE)
        Log.info(f"{self.__class__.__name__}: listen for discovered items on {self.topic_name_state}")
        self.sub_discovered_state = nmd.ros_node.create_subscription(
            ChangedState, self.topic_name_state, self._on_msg_state, qos_profile=qos_state_profile)
        Log.info(f"{self.__class__.__name__}: listen for endpoint items on {self.topic_name_endpoint}")
        self.sub_endpoints = nmd.ros_node.create_subscription(
            Endpoint, self.topic_name_endpoint, self._on_msg_endpoint, qos_profile=qos_endpoint_profile)
        self._lock_check = threading.RLock()
        self._thread_check_discovery_node = threading.Thread(target=self._check_discovery_node, daemon=True)
        self._thread_check_discovery_node.start()

    def _endpoints_to_provider(self, endpoints) -> List[RosProvider]:
        result = []
        for uri, endpoint in endpoints.items():
            origin = uri == nmd.launcher.server.uri
            provider = RosProvider(
                name=endpoint.name,
                host=get_hostname(endpoint.uri),
                port=get_port(endpoint.uri),
                origin=origin,
                hostnames=[get_hostname(endpoint.uri)])
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
            update_ros_state = False
            with self._lock_check:
                if self._ts_state_updated > self._ts_state_notified:
                    if time.time() - self._ts_state_notified > self._rate_check_discovery_node:
                        update_ros_state = True
            # as some services are called during the update, it may take some time
            if (update_ros_state or self._force_refresh) and self.websocket.count_clients() > 0 :
                self._force_refresh = False
                # trigger screen servicer to update
                nmd.launcher.server.screen_servicer.system_change()
                # participants should only be retrieved from discovery if they have also changed
                update_participants = self._update_participants
                self._update_participants = False
                ts_start_update = time.time()
                # create state
                state = self._state_jsonify.get_nodes_as_json(update_participants)
                with self._ros_node_list_mutex:
                    # set status only with lock, as this method runs in a thread
                    self._ros_node_list = state
                    self._ts_state_notified = ts_start_update
                    self.websocket.publish('ros.nodes.changed', {"timestamp": ts_start_update})

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

    def _on_msg_state(self, msg: ChangedState):
        '''
        The method to handle the received Log messages.
        :param msg: the received message
        :type msg: fkie_mas_msgs.ChangedState
        '''
        if not self.topic_state_publisher_count:
            self.topic_state_publisher_count = nmd.ros_node.count_publishers(
                self.topic_name_state)
            self.publish_discovery_state()
        # notify WebSocket clients, but not to often
        # notifications are sent from _check_discovery_node()
        if not (msg.topic_name.startswith("rq/") or msg.topic_name.startswith("rr/") or msg.topic_name == "ros_discovery_info") or msg.topic_name.endswith('/load_nodeReply') or msg.topic_name.endswith('/unload_nodeReply'):
            # ignore changes caused by service requests
            if msg.type == 0:
                # update participants on on next ros state update
                self._update_participants = True
            with self._lock_check:
                self._ts_state_updated = time.time()

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
        Log.info(f"{self.__class__.__name__}: Request to [ros.provider.get_list]")
        return json.dumps(self._endpoints_to_provider(self._endpoints), cls=SelfEncoder)

    def get_node_list(self, forceRefresh: bool = False) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.nodes.get_list]")
        node_list: List[RosNode] = self._get_ros_node_list(forceRefresh)
        return json.dumps(node_list, cls=SelfEncoder)

    def get_loggers(self, name: str) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.nodes.get_loggers] for '{name}', not implemented")
        loggerConfigs: List[LoggerConfig] = []
        return json.dumps({'result': False, 'logger': loggerConfigs, 'message': 'not implemented'}, cls=SelfEncoder)

    def set_logger_level(self, name: str, logger: List[LoggerConfig]) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.nodes.set_logger_level] for '{name}', not implemented")
        return json.dumps({'result': False, 'message': 'not implemented'}, cls=SelfEncoder)

    def stop_node(self, name: str) -> bool:
        Log.info(f"{self.__class__.__name__}: Request to stop node '{name}'")
        node: RosNode = self.get_ros_node(name)
        if node is None:
            node = self.get_ros_node_by_id(name)
        unloaded = False
        result = json.dumps({'result': False, 'message': f'{name} not found'}, cls=SelfEncoder)
        if node is not None:
            if node.container_name:
                unloaded = self.stop_composed_node(node)
            if not unloaded:
                result = nmd.launcher.server.screen_servicer.kill_node(node.name, signal.SIGTERM)
        if unloaded:
            result = json.dumps({'result': True, 'message': ''}, cls=SelfEncoder)
        nmd.launcher.server.screen_servicer.system_change()
        if node is not None:
            nmd.launcher.server.launch_servicer.node_stopped(node.name)
        return result

    def stop_subscriber(self, topic_name: str) -> bool:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.subscriber.stop]: {str(topic_name)}")
        ns, name = ros2_subscriber_nodename_tuple(topic_name)
        return self.stop_node(os.path.join(ns, name))

    def get_provider_timestamp(self, timestamp) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.provider.get_timestamp], timestamp: {timestamp}")
        return json.dumps({'timestamp': time.time() * 1000, "diff": time.time() * 1000 - float(timestamp)}, cls=SelfEncoder)

    def stop_composed_node(self, node: RosNode) -> bool:
        # try to unload node from container
        Log.info(f"{self.__class__.__name__}: -> unload '{node.name}' from '{node.container_name}'")
        # TODO: shutdown lifecycle nodes before unload
        try:
            container_node: RosNode = self.get_ros_node(node.container_name)
            if container_node is not None:
                unique_id_in_container = self.get_composed_node_id(node.container_name, node.name)
                if unique_id_in_container > -1:
                    service_unload_node = f'{container_node.name}/_container/unload_node'
                    Log.info(
                        f"{self.__class__.__name__}: -> unload '{node.name}' with id '{unique_id_in_container}' using service '{service_unload_node}'")
                    request = UnloadNode.Request()
                    request.unique_id = unique_id_in_container
                    response = nmd.launcher.call_service(
                        service_unload_node, UnloadNode, request)
                    if hasattr(response, "success") and response.success:
                        return True
                    elif not hasattr(response, "success"):
                        Log.warn(f"{self.__class__.__name__}: -> unload '{node.name}' error while call unload_node service")
                    else:
                        Log.warn(f"{self.__class__.__name__}: -> unload '{node.name}' error '{response.error_message}'")
                return False
            else:
                Log.warn(f"{self.__class__.__name__}: -> Container node '{node.container_name}' not found")
        except Exception as err:
            print(f"{err}")
        return False

    def get_composed_node_id(self, container_name: str, node_name: str) -> Number:
        service_list_nodes = f'{container_name}/_container/list_nodes'
        Log.debug(f"{self.__class__.__name__}: list nodes from '{service_list_nodes}'")
        request_list = ListNodes.Request()
        response_list = nmd.launcher.call_service(service_list_nodes, ListNodes, request_list)
        for name, unique_id in zip(response_list.full_node_names, response_list.unique_ids):
            if name == node_name:
                return unique_id
        return -1

    def _get_ros_node_list(self, forceRefresh: bool = False) -> List[RosNode]:
        # the status is updated in _check_discovery_node() in a thread
        # in the meantime, the cached list is returned
        # after the state is ready, a 'ros.nodes.changed' notification will be send
        if self._ros_node_list is None or forceRefresh:
            self._force_refresh = True
            self._ros_node_list = []
        return self._ros_node_list

    def get_ros_node(self, node_name: str) -> Union[RosNode, None]:
        node_list: List[RosNode] = self._get_ros_node_list()
        for node in node_list:
            if node_name == node.name:
                return node
        return None

    def get_ros_node_by_id(self, node_id: str) -> Union[RosNode, None]:
        node_list: List[RosNode] = self._get_ros_node_list()
        for node in node_list:
            if node_id == node.id:
                return node
        return None
