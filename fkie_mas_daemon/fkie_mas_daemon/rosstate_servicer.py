
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from fkie_mas_daemon.monitor_servicer import MonitorServicer
from fkie_mas_daemon.rosstate_jsonify import RosStateJsonify
from fkie_mas_daemon.rosstate_jsonify import ServiceNameWoPrefix
from fkie_mas_daemon.rosstate_jsonify import ServiceType
from fkie_mas_daemon.rosstate_jsonify import TopicNameWoPrefix
from fkie_mas_daemon.rosstate_jsonify import TopicType
import fkie_mas_daemon as nmd
from fkie_mas_msgs.msg import Endpoint
from fkie_mas_msgs.msg import ParticipantEntitiesInfo
from fkie_mas_msgs.msg import Participants
from fkie_mas_msgs.msg import ChangedState
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_mas_pylib.websocket.server import WebSocketServer
from fkie_mas_pylib.system.url import get_port
from fkie_mas_pylib.system.host import get_hostname
from fkie_mas_pylib.system.host import get_host_name
from fkie_mas_pylib.system.host import get_local_addresses
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_mas_pylib.defines import NM_NAMESPACE
from fkie_mas_pylib.defines import NM_DISCOVERY_NAME
from fkie_mas_pylib.interface.launch_interface import LaunchContent
from fkie_mas_pylib.interface.runtime_interface import LoggerConfig
from fkie_mas_pylib.interface.runtime_interface import RosComposable
from fkie_mas_pylib.interface.runtime_interface import RosLifecycleState
from fkie_mas_pylib.interface.runtime_interface import RosService
from fkie_mas_pylib.interface.runtime_interface import RosTopicId
from fkie_mas_pylib.interface.runtime_interface import RosTopic
from fkie_mas_pylib.interface.runtime_interface import RosNode
from fkie_mas_pylib.interface.runtime_interface import RosProvider
from fkie_mas_pylib.interface.runtime_interface import SystemWarning
from fkie_mas_pylib.interface.runtime_interface import SystemWarningGroup
from fkie_mas_pylib.interface import SelfEncoder
from typing import Dict
from typing import List
from numbers import Number
from typing import Text
from typing import Tuple
from typing import Union

import os
import json
import signal
import socket
import sys
import threading
import time

from composition_interfaces.srv import ListNodes
from composition_interfaces.srv import UnloadNode
HAS_LOGGER_INTERFACE = False
try:
    from rcl_interfaces.srv import GetLoggerLevels
    from rcl_interfaces.srv import SetLoggerLevels
    from rcl_interfaces.msg import LoggerLevel
    from rcl_interfaces.msg import SetLoggerLevelsResult
    HAS_LOGGER_INTERFACE = True
except:
    print("Can't include rcl_interfaces.srv.GetLoggerLevels: logger interface disabled!")


RATE_CHECK_DISCOVERY_NODE_HZ = 1.0


class RosStateServicer:

    def __init__(self, websocket: WebSocketServer, monitor_servicer: MonitorServicer = None, endpoint_notification_interval: int = 61.0, test_env=False):
        Log.info("Create ros_state servicer")
        self._endpoint_timeout_sec = endpoint_notification_interval * 2.0 + 1.0
        self._endpoints: Dict[str, Endpoint] = {}  # uri : Endpoint
        self._endpoints_ts: Dict[str, float] = {}  # uri : timestamp
        self._ros_node_list: List[RosNode] = []
        self._ros_node_list_str: str = json.dumps(self._ros_node_list, cls=SelfEncoder)
        self._ros_topic_list: List[RosNode] = []
        self._ros_topic_list_str: str = json.dumps(self._ros_topic_list, cls=SelfEncoder)
        self._ros_service_list: List[RosNode] = []
        self._ros_service_list_str: str = json.dumps(self._ros_service_list, cls=SelfEncoder)
        self._ros_service_dict: Dict[Tuple[ServiceNameWoPrefix, ServiceType], RosService] = {}
        self._ros_topic_dict: Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic] = {}
        self._ros_node_list_mutex = threading.RLock()
        self._ros_topic_list_mutex = threading.RLock()
        self._ros_service_list_mutex = threading.RLock()
        self._ros_lifecycle_mutex = threading.RLock()
        self._ros_composable_mutex = threading.RLock()
        self.topic_name_state = f"{NM_NAMESPACE}/{NM_DISCOVERY_NAME}/changed"
        self.topic_name_participants = f"{NM_NAMESPACE}/{NM_DISCOVERY_NAME}/participants"
        self.topic_name_endpoint = f"{NM_NAMESPACE}/daemons"
        self.topic_state_publisher_count = 0
        self._force_refresh = False
        self._ts_state_updated = 0
        self._ts_state_notified = 0
        self._last_seen_participant_count = 0
        self._thread_check_discovery_node = None
        self._check_delay = 1.0 / RATE_CHECK_DISCOVERY_NODE_HZ
        self._on_shutdown = False
        self._state_jsonify = RosStateJsonify(cb_nodes=self._callback_nodes,
                                              cb_topics=self._callback_topics,
                                              cb_services=self._callback_services,
                                              cb_composables=self._callback_composable_nodes,
                                              cb_lifecycle=self._callback_lifecycle_state,
                                              monitor_servicer=monitor_servicer)
        self.websocket = websocket
        self.monitor_servicer = monitor_servicer
        self._lifecycle_state: List[RosLifecycleState] = []
        self._composables_nodes: List[RosComposable] = []
        self._discovered_nodes_count = 0
        self._topic_types = ""
        self._timestamp = 0
        websocket.register("ros.provider.get_list", self.get_provider_list)
        websocket.register("ros.nodes.get_list", self.get_node_list)
        websocket.register("ros.services.get_list", self.get_service_list)
        websocket.register("ros.topics.get_list", self.get_topic_list)
        websocket.register("ros.nodes.get_loggers", self.get_loggers)
        websocket.register("ros.nodes.set_logger_level", self.set_logger_level)
        websocket.register("ros.nodes.stop_node", self.stop_node)
        websocket.register("ros.nodes.get_lifecycle", self.get_lifecycle)
        websocket.register("ros.nodes.get_composable", self.get_composable)
        websocket.register("ros.subscriber.stop", self.stop_subscriber)
        websocket.register("ros.provider.get_timestamp", self.get_provider_timestamp)

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
        qos_participants_profile = QoSProfile(depth=10,
                                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                              history=QoSHistoryPolicy.KEEP_LAST,
                                              reliability=QoSReliabilityPolicy.RELIABLE)
        Log.info(f"{self.__class__.__name__}: listen for discovered items on {self.topic_name_state}")
        self.sub_discovered_state = nmd.ros_node.create_subscription(
            ChangedState, self.topic_name_state, self._on_msg_state, qos_profile=qos_state_profile)
        Log.info(f"{self.__class__.__name__}: listen for endpoint items on {self.topic_name_endpoint}")
        self.sub_endpoints = nmd.ros_node.create_subscription(
            Endpoint, self.topic_name_endpoint, self._on_msg_endpoint, qos_profile=qos_endpoint_profile)
        self.sub_participants = nmd.ros_node.create_subscription(
            Participants, self.topic_name_participants, self._on_msg_participants, qos_profile=qos_participants_profile)
        self._lock_check = threading.RLock()
        self._thread_check_discovery_node = threading.Thread(target=self._check_discovery_node, daemon=True)
        self._thread_check_discovery_node.start()

    def get_lifecycle(self) -> List[RosLifecycleState]:
        with self._ros_lifecycle_mutex:
            return self._lifecycle_state

    def get_composable(self) -> List[RosComposable]:
        with self._ros_composable_mutex:
            return self._composables_nodes

    def _callback_lifecycle_state(self, states: List[RosLifecycleState]):
        with self._ros_lifecycle_mutex:
            # remove from current list
            filtered_list = []
            for state in self._lifecycle_state:
                if len([state for lc in states if lc.id == state.id]) == 0:
                    filtered_list.append(state)
            filtered_list.extend(states)
            self._lifecycle_state = filtered_list
            self.websocket.publish('ros.nodes.lifecycle', {"lifecycle": states})

    def _callback_composable_nodes(self, composables: List[RosComposable]):
        with self._ros_composable_mutex:
            # remove from current list
            filtered_list = []
            for state in self._composables_nodes:
                if len([state for cm in composables if cm.nodeId == state.nodeId]) == 0:
                    filtered_list.append(state)
            filtered_list.extend(composables)
            self._composables_nodes = filtered_list
            self.websocket.publish('ros.nodes.composable', {"composable": composables})

    def _callback_nodes(self, nodes: List[RosNode]):
        new_nodes_str = json.dumps(nodes, cls=SelfEncoder)
        with self._ros_node_list_mutex:
            self._ros_node_list = nodes
            self._ts_state_notified = time.time()
            if self._ros_node_list_str != new_nodes_str:
                self._ros_node_list_str = new_nodes_str
                Log.debug(f"new node list; size: {sys.getsizeof(new_nodes_str) / 1024 / 1024:,.4f} Mbit")
                self.websocket.publish('ros.nodes.changed', {"timestamp": self._ts_state_notified})
                # update local nodes of the monitor servicer
                self.monitor_servicer.update_local_node_names(self._state_jsonify.get_local_node_names())

    def _callback_topics(self, topics: Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic]):
        new_topic_str = json.dumps([v for v in topics.values()], cls=SelfEncoder)
        with self._ros_topic_list_mutex:
            self._ros_topic_list = topics
            self._ts_state_notified = time.time()
            if self._ros_topic_list_str != new_topic_str:
                self._ros_topic_list_str = new_topic_str
                Log.debug(f"new topics list; size: {sys.getsizeof(new_topic_str) / 1024 / 1024:,.4f} Mbit")
                self.websocket.publish('ros.topics.changed', {"timestamp": self._ts_state_notified})

    def _callback_services(self, services: Dict[Tuple[ServiceNameWoPrefix, ServiceType], RosService]):
        new_service_str = json.dumps([v for v in services.values()], cls=SelfEncoder)
        with self._ros_service_list_mutex:
            self._ros_service_list = services
            self._ts_state_notified = time.time()
            if self._ros_service_list_str != new_service_str:
                self._ros_service_list_str = new_service_str
                Log.debug(f"new services list; size: {sys.getsizeof(new_service_str) / 1024 / 1024:,.4f} Mbit")
                self.websocket.publish('ros.services.changed', {"timestamp": self._ts_state_notified})

    def _endpoints_to_provider(self, endpoints) -> List[RosProvider]:
        result = []
        local_hostname = get_host_name()
        w_resolve_failed = SystemWarningGroup(SystemWarningGroup.ID_RESOLVE_FAILED)
        for uri, endpoint in endpoints.items():
            origin = uri == nmd.launcher.server.uri
            hostname = get_hostname(endpoint.uri)
            hostnames = [hostname]
            if hostname == local_hostname:
                hostnames.extend(get_local_addresses())
            else:
                try:
                    hostnames.append(socket.gethostbyname(hostname))
                except Exception as err:
                    details = f"{self.__class__.__name__}: socket.gethostbyname({hostname}): {err}"
                    Log.warn(details)
                    w_resolve_failed.append(SystemWarning(msg=f"unknown host: {hostname}", details=details))
                    hostnames.append(hostname)
            provider = RosProvider(
                name=endpoint.name,
                host=hostname,
                port=get_port(endpoint.uri),
                origin=origin,
                hostnames=list(set(hostnames))
            )
            result.append(provider)
        self.monitor_servicer.update_warning_groups([w_resolve_failed])
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
                if nmd.ros_node.count_publishers(self.topic_name_state) == 0 and nmd.ros_node.count_publishers(self.topic_name_participants) == 0:
                    self.topic_state_publisher_count = 0
                    self.publish_discovery_state()
                    with self._lock_check:
                        self._ts_state_updated = time.time()
            # If a change was detected by discovery node we received _on_msg_state().
            # Therefor the self._ts_state_updated was updated.
            # But we delay the check for changes by
            update_ros_state = False
            with self._lock_check:
                if self._ts_state_updated > self._ts_state_notified:
                    if time.time() - self._ts_state_notified > self._check_delay:
                        update_ros_state = True
            send_notification = False
            # send only if websocket clients are connected
            if (update_ros_state or self._force_refresh) and self.websocket.count_clients() > 0:
                send_notification = True

            if send_notification:
                try:
                    # trigger screen servicer to update
                    nmd.launcher.server.screen_servicer.system_change()
                    # trigger the update of the ros state
                    # The updates are received by callback defined in the __init__()
                    self._state_jsonify.update_state(self._force_refresh)
                    with self._ros_node_list_mutex:
                        self._force_refresh = False
                except Exception:
                    import traceback
                    msg = traceback.format_exc()
                    Log.warn(msg)
                    warnings_group: SystemWarningGroup = SystemWarningGroup(SystemWarningGroup.ID_ROS_STATE)
                    warnings_group.append(SystemWarning(msg=msg))
                    self.monitor_servicer.update_warning_groups([warnings_group])

            # check time jumps
            now = time.time()
            if now < self._timestamp:
                time_jump_msg = "Time jump into past detected! Restart all ROS nodes, includes MAS nodes, please!"
                Log.warn(time_jump_msg)
                w_time_jump = SystemWarningGroup(SystemWarningGroup.ID_TIME_JUMP)
                w_time_jump.append(SystemWarning(msg='Timejump into past detected!',
                                   hint='Restart all ROS nodes, includes master_discovery, please! master_discovery shutting down in 5 seconds!'))
                self.monitor_servicer.update_warning_groups([w_time_jump])
            else:
                self._timestamp = now
            # check for timeouted provider
            with self._lock_check:
                removed_uris = []
                for uri, ts in self._endpoints_ts.items():
                    if now - ts > self._endpoint_timeout_sec:
                        Log.info(f"{self.__class__.__name__}: remove outdated daemon {uri}, not seen for {now - ts} sec")
                        if uri in self._endpoints:
                            del self._endpoints[uri]
                        removed_uris.append(uri)
                for uri in removed_uris:
                    del self._endpoints_ts[uri]
                if len(removed_uris) > 0:
                    self._publish_masters()
            time.sleep(self._check_delay)

    def stop(self):
        '''
        Unregister the subscribed topic
        '''
        self._on_shutdown = True
        self._state_jsonify.stop()
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
        The method to handle the DDS changes.
        :param msg: the received message
        :type msg: fkie_mas_msgs.ChangedState
        '''
        if not self.topic_state_publisher_count:
            self.topic_state_publisher_count = nmd.ros_node.count_publishers(
                self.topic_name_state)
            self.publish_discovery_state()
        with self._lock_check:
            self._ts_state_updated = time.time()

    def _on_msg_participants(self, msg: Participants):
        '''
        The method to handle the Participants.
        :param msg: the received message
        :type msg: fkie_mas_msgs.Participants
        '''
        with self._ros_node_list_mutex:
            self._state_jsonify.apply_participants(msg)
        if not self.topic_state_publisher_count:
            self.topic_state_publisher_count = 1
            self.publish_discovery_state()

    def _on_msg_endpoint(self, msg: Endpoint):
        '''
        The method to handle the received Endpoints messages.
        :param msg: the received message
        :type msg: fkie_mas_msgs.Endpoint<XXX>
        '''
        Log.debug(
            f"{self.__class__.__name__}: new endpoint on {self.topic_name_endpoint}: {msg.uri}")
        is_new = False
        with self._lock_check:
            if msg.on_shutdown:
                if msg.uri in self._endpoints:
                    is_new = True
                    Log.info(f"{self.__class__.__name__}: remove daemon {msg.uri}")
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
            self._endpoints_ts[msg.uri] = time.time()

    def get_provider_list(self) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.provider.get_list]")
        with self._lock_check:
            return json.dumps(self._endpoints_to_provider(self._endpoints), cls=SelfEncoder)

    def get_node_list(self, forceRefresh: bool = False) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.nodes.get_list]; forceRefresh: {forceRefresh}")
        with self._ros_node_list_mutex:
            self._get_ros_node_list(forceRefresh)
            return self._ros_node_list_str

    def _topic_in_filter(self, topic_name: str, topic_type: str, filter: List[RosTopicId]) -> bool:
        if len(filter) == 0:
            return True
        for ft in filter:
            name_ok = True
            type_ok = True
            if ft.name:
                name_ok = topic_name == ft.name
            if ft.msg_type:
                type_ok = topic_type == ft.msg_type
            if name_ok and type_ok:
                return True
        return False

    def get_service_list(self, filter: List[RosTopicId] = []) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.topics.get_list]")
        with self._ros_node_list_mutex:
            filtered: List[RosService] = []
            if len(filter) > 0:
                for id, service in self._ros_service_dict.items():
                    if self._topic_in_filter(id[0], id[1], filter):
                        filtered.append(service)
                return json.dumps(filtered, cls=SelfEncoder)
            return self._ros_service_list_str

    def get_topic_list(self, filter: List[RosTopicId] = []) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.services.get_list]")
        with self._ros_node_list_mutex:
            filtered: List[RosTopic] = []
            if len(filter) > 0:
                for id, topic in self._ros_topic_dict.items():
                    if self._topic_in_filter(id[0], id[1], filter):
                        filtered.append(topic)
                return json.dumps(filtered, cls=SelfEncoder)
            return self._ros_topic_list_str

    def get_loggers(self, name: str, loggers: List[str] = []) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.nodes.get_loggers] for '{name}', loggers: {loggers}")
        if not HAS_LOGGER_INTERFACE:
            raise Exception("ros2 version on this client does not support logger interface!")
        logger_names = loggers
        if not logger_names or len(loggers) == 0:
            logger_names = [name.replace("/", ".").strip("."), "rcl"]
        loggerConfigs: List[LoggerConfig] = []
        # get logger names if loggers list is empty
        if len(loggers) == 0:
            try:
                service_name = '%s/logger_list' % name
                request_list = GetLoggerLevels.Request()
                request_list.names = []
                get_logger = nmd.launcher.call_service(service_name, GetLoggerLevels, request_list)
                if get_logger:
                    for logger in get_logger.levels:
                        logger_names.append(logger.name)
            except:
                pass
        # get current logger levels
        service_name = '%s/get_logger_levels' % name
        request_list = GetLoggerLevels.Request()
        request_list.names = logger_names
        get_logger = nmd.launcher.call_service(service_name, GetLoggerLevels, request_list)
        if get_logger:
            for logger in get_logger.levels:
                loggerConfigs.append(LoggerConfig(
                    level=LoggerConfig.LogLevelType.fromRos2(logger.level), name=logger.name))
        return json.dumps(loggerConfigs, cls=SelfEncoder)

    def set_logger_level(self, name: str, loggers: List[LoggerConfig]) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.nodes.set_logger_level] for '{name}'")
        if not HAS_LOGGER_INTERFACE:
            raise Exception("ros2 version on this client does not support logger interface!")
        # request the current logger
        service_name_get = '%s/set_logger_levels' % name
        request_set = SetLoggerLevels.Request()
        for logger in loggers:
            log_level = LoggerLevel()
            log_level.name = logger.name
            log_level.level = LoggerConfig.LogLevelType.toRos2(logger.level)
            request_set.levels.append(log_level)
        set_logger = nmd.launcher.call_service(service_name_get, SetLoggerLevels, request_set)
        result = True
        reason = ""
        if set_logger:
            idx = 0
            for set_res in set_logger.results:
                if not set_res.successful:
                    result = False
                    reason += f"{idx}: {set_res.reason}; "
                idx += 1
        else:
            result = False
        return json.dumps({'result': result, 'message': reason}, cls=SelfEncoder)

    def stop_node(self, name: str) -> bool:
        Log.info(f"{self.__class__.__name__}: Request to stop node '{name}'")
        node: RosNode = self.get_ros_node(name)
        if node is None:
            node = self.get_ros_node_by_id(name)
        unloaded = False
        result = json.dumps({'result': False, 'message': f'{name} not found'}, cls=SelfEncoder)
        if node is not None:
            if not node.container_name:
                # composable node are determine by service call.
                # we have to search in composable state for the node
                with self._ros_composable_mutex:
                    for state in self._composables_nodes:
                        if node.name in state.nodes:
                            node.container_name = state.containerName
                            break
            if node.container_name:
                unloaded = self.stop_composed_node(node)
            if not unloaded:
                # it was not a composable node -> try to stop
                result = nmd.launcher.server.screen_servicer.kill_node(node.name, signal.SIGTERM)
        if unloaded:
            result = json.dumps({'result': True, 'message': ''}, cls=SelfEncoder)
        nmd.launcher.server.screen_servicer.system_change()
        if node is not None:
            nmd.launcher.server.launch_servicer.node_stopped(node.name)
        return result

    def stop_subscriber(self, topic_name: str) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.subscriber.stop]: {str(topic_name)}")
        ns, name = ros2_subscriber_nodename_tuple(topic_name)
        result = self.stop_node(os.path.join(ns, name))
        return json.dumps({"result": result, "message": ""}, cls=SelfEncoder)

    def get_provider_timestamp(self, timestamp) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.provider.get_timestamp], timestamp: {timestamp}")
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
            Log.warn(f"{err}")
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
        if forceRefresh:
            self._force_refresh = True
        if (self._ros_node_list is None or forceRefresh):
            self._ros_node_list = []
            self._ros_node_list_str = json.dumps(self._ros_node_list, cls=SelfEncoder)
            self._ros_service_dict = {}
            self._ros_topic_dict = {}
        return self._ros_node_list

    def get_ros_node(self, node_name: str) -> Union[RosNode, None]:
        with self._ros_node_list_mutex:
            node_list: List[RosNode] = self._get_ros_node_list()
            for node in node_list:
                if node_name == node.name:
                    return node
            return None

    def get_ros_node_by_id(self, node_id: str) -> Union[RosNode, None]:
        with self._ros_node_list_mutex:
            node_list: List[RosNode] = self._get_ros_node_list()
            for node in node_list:
                if node_id == node.id:
                    return node
            return None
