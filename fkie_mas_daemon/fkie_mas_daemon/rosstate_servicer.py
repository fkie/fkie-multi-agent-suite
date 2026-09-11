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
from fkie_mas_msgs.msg import Participants
from fkie_mas_msgs.msg import ChangedState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_mas_pylib.websocket.server import WebSocketServer
from fkie_mas_pylib.system.url import get_port
from fkie_mas_pylib.system.host import get_hostname
from fkie_mas_pylib.system.host import get_host_name
from fkie_mas_pylib.system.host import get_local_addresses
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.defines import ros2_publisher_nodename_tuple
from fkie_mas_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_mas_pylib.defines import ros2_action_nodename_tuple
from fkie_mas_pylib.defines import ros2_action_introspection_nodename_tuple
from fkie_mas_pylib.defines import ros2_service_introspection_nodename_tuple
from fkie_mas_pylib.defines import NM_NAMESPACE
from fkie_mas_pylib.defines import NM_DISCOVERY_NAME
from fkie_mas_pylib.interface.runtime_interface import DelayRosUpdateState
from fkie_mas_pylib.interface.runtime_interface import LifecycleTransition
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
from typing import Tuple
from typing import Union

import os
import json
import signal
import socket
import sys
import threading
import time
import traceback
import rclpy

from composition_interfaces.srv import ListNodes
from composition_interfaces.srv import UnloadNode
HAS_LOGGER_INTERFACE = False
try:
    from rcl_interfaces.srv import GetLoggerLevels
    from rcl_interfaces.srv import SetLoggerLevels
    from rcl_interfaces.msg import LoggerLevel
    HAS_LOGGER_INTERFACE = True
except ImportError:
    print("Can't include rcl_interfaces.srv.GetLoggerLevels: logger interface disabled!")

HAS_LIFECYCLE_INTERFACE = False
try:
    from lifecycle_msgs.srv import GetState
    from lifecycle_msgs.srv import GetAvailableTransitions
    HAS_LIFECYCLE_INTERFACE = True
except ImportError:
    print("Can't include lifecycle_msgs.srv.GetState: lifecycle interface disabled!")


RATE_CHECK_DISCOVERY_NODE_HZ = 0.5


class RosStateServicer:

    def __init__(self, websocket: WebSocketServer, monitor_servicer: MonitorServicer = None, endpoint_notification_interval: float = 61.0, test_env=False):
        Log.info("Create ros_state servicer")
        self._endpoint_timeout_sec = endpoint_notification_interval * 2.0 + 1.0
        self._endpoints: Dict[str, Endpoint] = {}  # uri : Endpoint
        self._endpoints_ts: Dict[str, float] = {}  # uri : timestamp
        self._ros_node_list: List[RosNode] = []
        self._ros_node_list_str: str = json.dumps(self._ros_node_list, cls=SelfEncoder)
        self._ros_topic_list_str: str = json.dumps([], cls=SelfEncoder)
        self._ros_service_list_str: str = json.dumps([], cls=SelfEncoder)
        self._ros_service_name_list: List[str] = []
        self._ros_service_name_set: set = set()
        self._ros_service_dict: Dict[Tuple[ServiceNameWoPrefix, ServiceType], RosService] = {}
        self._ros_topic_dict: Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic] = {}
        self._count_nodes = 0
        self._count_topics = 0
        self._count_services = 0
        self._ros_node_state_mutex = threading.RLock()
        self._ros_topic_state_mutex = threading.RLock()
        self._ros_service_state_mutex = threading.RLock()
        self._ros_lifecycle_mutex = threading.RLock()
        self._ros_composable_mutex = threading.RLock()
        # this lock protects the endpoints and the state timestamps.
        # it has to be created here, because messages can be received before start() was called.
        self._lock_check = threading.RLock()
        self._shutdown_event = threading.Event()
        self._is_dds = os.environ.get("RMW_IMPLEMENTATION", "") != "rmw_zenoh_cpp"
        self.topic_name_state = f"{NM_NAMESPACE}/{NM_DISCOVERY_NAME}/changed"
        self.topic_name_participants = ""
        if self._is_dds:
            self.topic_name_participants = f"{NM_NAMESPACE}/{NM_DISCOVERY_NAME}/participants"
        self.topic_name_endpoint = f"{NM_NAMESPACE}/daemons"
        self.topic_state_publisher_count = 0
        self._force_refresh = False
        self._had_clients = False
        self._ts_state_updated = 0
        self._ts_state_notified = 0
        self._ts_delay_until = 0
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
        self._callback_group_logger = MutuallyExclusiveCallbackGroup()
        self._callback_group_composed = ReentrantCallbackGroup()
        self._callback_group_lifecycle = ReentrantCallbackGroup()
        websocket.register("ros.provider.get_list", self.get_provider_list)
        websocket.register("ros.nodes.get_list", self.get_node_list)
        websocket.register("ros.services.get_list", self.get_service_list)
        websocket.register("ros.topics.get_list", self.get_topic_list)
        websocket.register("ros.nodes.get_loggers", self.get_loggers)
        websocket.register("ros.nodes.set_logger_level", self.set_logger_level)
        websocket.register("ros.nodes.stop_node", self.stop_node)
        websocket.register("ros.nodes.get_lifecycle", self.get_lifecycle)
        websocket.register("ros.nodes.update_lifecycle", self.update_lifecycle)
        websocket.register("ros.nodes.get_composable", self.get_composable)
        websocket.register("ros.publisher.stop", self.stop_publisher)
        websocket.register("ros.publisher.has", self.has_publisher)
        websocket.register("ros.subscriber.stop", self.stop_subscriber)
        websocket.register("ros.action.stop", self.stop_action)
        websocket.register("ros.action.introspection.stop", self.stop_action_introspection)
        websocket.register("ros.service.introspection.stop", self.stop_service_introspection)
        websocket.register("ros.provider.get_timestamp", self.get_provider_timestamp)
        websocket.subscribe("ros.daemon.delay_update_state", self.delay_update_state)

    def start(self):
        qos_state_profile = QoSProfile(depth=10,
                                       #    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                       #    history=QoSHistoryPolicy.KEEP_LAST,
                                       #    reliability=QoSReliabilityPolicy.RELIABLE
                                       )
        qos_endpoint_profile = QoSProfile(depth=1,
                                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                          # history=QoSHistoryPolicy.KEEP_LAST,
                                          reliability=QoSReliabilityPolicy.RELIABLE)
        qos_participants_profile = QoSProfile(depth=1,
                                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                              history=QoSHistoryPolicy.KEEP_LAST,
                                              reliability=QoSReliabilityPolicy.RELIABLE)
        Log.info(f"{self.__class__.__name__}: listen for discovered items on {self.topic_name_state}")
        self.sub_discovered_state = nmd.ros_node.create_subscription(
            ChangedState, self.topic_name_state, self._on_msg_state, qos_profile=qos_state_profile)
        Log.info(f"{self.__class__.__name__}: listen for endpoint items on {self.topic_name_endpoint}")
        self.sub_endpoints = nmd.ros_node.create_subscription(
            Endpoint, self.topic_name_endpoint, self._on_msg_endpoint, qos_profile=qos_endpoint_profile)
        if self._is_dds:
            Log.info(f"{self.__class__.__name__}: listen for participants on {self.topic_name_participants}")
            self.sub_participants = nmd.ros_node.create_subscription(
                Participants, self.topic_name_participants, self._on_msg_participants, qos_profile=qos_participants_profile)
        self._thread_check_discovery_node = threading.Thread(target=self._check_discovery_node, daemon=True)
        self._thread_check_discovery_node.start()

    def get_lifecycle(self) -> List[RosLifecycleState]:
        with self._ros_lifecycle_mutex:
            # return a shallow copy to avoid concurrent modification while serializing
            return list(self._lifecycle_state)

    def update_lifecycle(self, node_name: str = None) -> None:
        if node_name is None:
            return
        if not HAS_LIFECYCLE_INTERFACE:
            raise Exception("ros2 version on this client does not support lifecycle interface!")

        lifecycle_state = RosLifecycleState(node_name, node_name)
        node: RosNode = self.get_ros_node(node_name)
        if node is None:
            node = self.get_ros_node_by_id(node_name)
        if node is None:
            return
        # initialize the service name, it is used in the exception handler
        service_name = f'{node.name}/get_state'
        try:
            service_available = False
            with self._ros_service_state_mutex:
                service_available = service_name in self._ros_service_name_set
            if service_available:
                Log.debug(f"{self.__class__.__name__}: updated lifecycle state for '{service_name}'")
                request_state = GetState.Request()
                get_state = nmd.launcher.call_service(
                    service_name, GetState, request_state, callback_group=self._callback_group_lifecycle)
                if get_state:
                    lifecycle_state.state = get_state.current_state.label
            if lifecycle_state.state != "unknown":
                # skip if the sate was not successful
                service_name = f'{node.name}/get_available_transitions'
                service_available = False
                with self._ros_service_state_mutex:
                    service_available = service_name in self._ros_service_name_set
                if service_available:
                    Log.debug(f"{self.__class__.__name__}: updated lifecycle state for '{service_name}'")
                    request_state = GetAvailableTransitions.Request()
                    response = nmd.launcher.call_service(
                        service_name, GetAvailableTransitions, request_state, callback_group=self._callback_group_lifecycle)
                    if response:
                        for transition in response.available_transitions:
                            lifecycle_state.available_transitions.append(
                                LifecycleTransition(transition.transition.label, transition.transition.id))
            self._callback_lifecycle_state([lifecycle_state])
        except Exception as e:
            Log.warn(f"{self.__class__.__name__}: failed updated lifecycle state for '{service_name}': {e}")

    def get_composable(self) -> List[RosComposable]:
        with self._ros_composable_mutex:
            # return a shallow copy to avoid concurrent modification while serializing
            return list(self._composables_nodes)

    def _callback_lifecycle_state(self, states: List[RosLifecycleState]):
        with self._ros_lifecycle_mutex:
            # remove from current list
            updated_ids = {lc.id for lc in states}
            filtered_list = [state for state in self._lifecycle_state if state.id not in updated_ids]
            filtered_list.extend(states)
            self._lifecycle_state = filtered_list
        # publish outside of the lock, the websocket call can block
        self.websocket.publish('ros.nodes.lifecycle', {"lifecycle": states})

    def _callback_composable_nodes(self, composables: List[RosComposable]):
        with self._ros_composable_mutex:
            # remove from current list
            updated_ids = {cm.nodeId for cm in composables}
            filtered_list = [state for state in self._composables_nodes if state.nodeId not in updated_ids]
            filtered_list.extend(composables)
            self._composables_nodes = filtered_list
        # publish outside of the lock, the websocket call can block
        self.websocket.publish('ros.nodes.composable', {"composable": composables})

    def _callback_nodes(self, nodes: List[RosNode]):
        new_nodes_str = json.dumps(nodes, cls=SelfEncoder)
        ts_notified = time.time()
        changed = False
        with self._ros_node_state_mutex:
            # self._count_nodes = len({n.name for n in nodes})
            self._ros_node_list = nodes
            if self._ros_node_list_str != new_nodes_str:
                self._ros_node_list_str = new_nodes_str
                changed = True
        self._set_state_notified(ts_notified)
        if changed:
            Log.debug(f"new node list; size: {sys.getsizeof(new_nodes_str) / 1024 / 1024:,.4f} Mbit")
            self.websocket.publish('ros.nodes.changed', {"timestamp": ts_notified})
            # update local nodes of the monitor servicer
            if self.monitor_servicer is not None:
                self.monitor_servicer.update_local_node_names(self._state_jsonify.get_local_node_names())
            # update ionotify for node executables
            alive = {n.name for n in nodes}
            nmd.launcher.server.launch_servicer.reconcile_running_nodes(alive)

    def _callback_topics(self, topics: Dict[Tuple[TopicNameWoPrefix, TopicType], RosTopic]):
        new_topic_str = json.dumps([v for v in topics.values()], cls=SelfEncoder)
        ts_notified = time.time()
        changed = False
        with self._ros_topic_state_mutex:
            # self._count_topics = len(topics)
            self._ros_topic_dict = topics
            if self._ros_topic_list_str != new_topic_str:
                self._ros_topic_list_str = new_topic_str
                changed = True
        self._set_state_notified(ts_notified)
        if changed:
            Log.debug(f"new topics list; size: {sys.getsizeof(new_topic_str) / 1024 / 1024:,.4f} Mbit")
            self.websocket.publish('ros.topics.changed', {"timestamp": ts_notified})

    def _callback_services(self, services: Dict[Tuple[ServiceNameWoPrefix, ServiceType], RosService]):
        new_service_str = json.dumps([v for v in services.values()], cls=SelfEncoder)
        ts_notified = time.time()
        changed = False
        with self._ros_service_state_mutex:
            # self._count_services = len(services)
            self._ros_service_dict = services
            if self._ros_service_list_str != new_service_str:
                self._ros_service_list_str = new_service_str
                self._ros_service_name_list = [key[0] for key in services.keys()]
                # keep a set for fast membership tests
                self._ros_service_name_set = set(self._ros_service_name_list)
                changed = True
        self._set_state_notified(ts_notified)
        if changed:
            Log.debug(f"new services list; size: {sys.getsizeof(new_service_str) / 1024 / 1024:,.4f} Mbit")
            self.websocket.publish('ros.services.changed', {"timestamp": ts_notified})

    def _set_state_notified(self, timestamp: float) -> None:
        '''
        Stores the timestamp of the last notification. It is protected by the same
        lock as self._ts_state_updated to keep both values consistent.
        '''
        with self._lock_check:
            self._ts_state_notified = timestamp

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
        if self.monitor_servicer is not None:
            self.monitor_servicer.update_warning_groups([w_resolve_failed])
        return result

    def _publish_masters(self, endpoints: Dict[str, Endpoint] = None):
        # this method must not be called while self._lock_check is hold,
        # the name resolution in _endpoints_to_provider() can block for a long time
        if endpoints is None:
            with self._lock_check:
                endpoints = dict(self._endpoints)
        result = self._endpoints_to_provider(endpoints)
        self.websocket.publish('ros.provider.list', result)

    def publish_discovery_state(self):
        self.websocket.publish('ros.discovery.ready', {
            'status': self.topic_state_publisher_count > 0, 'timestamp': time.time() * 1000})

    def get_publisher_count(self):
        if hasattr(self, 'topic_name_endpoint') and self.topic_name_endpoint is not None:
            return nmd.ros_node.count_publishers(self.topic_name_endpoint)
        return -1

    def _check_discovery_node(self):
        while not self._on_shutdown and rclpy.ok():
            try:
                now = time.time()
                if now < self._ts_delay_until and not self._force_refresh:
                    continue
                if self.topic_state_publisher_count:
                    # check if we have a discovery node
                    if nmd.ros_node.count_publishers(self.topic_name_state) == 0:
                        self.topic_state_publisher_count = 0
                        self.publish_discovery_state()
                        with self._lock_check:
                            self._ts_state_updated = time.time()
                # If a change was detected by discovery node we received _on_msg_state().
                # Therefor the self._ts_state_updated was updated.
                # But we delay the check for changes by
                update_ros_state = False
                with self._lock_check:
                    ts_state_notified = self._ts_state_notified
                    if self._ts_state_updated > ts_state_notified:
                        if now - ts_state_notified > self._check_delay:
                            update_ros_state = True
                clients_connected = self.websocket.count_clients() > 0
                with self._ros_node_state_mutex:
                    if clients_connected and not self._had_clients:
                        self._force_refresh = True
                    force_refresh = self._force_refresh
                self._had_clients = clients_connected
                send_notification = False
                # send only if websocket clients are connected
                if (update_ros_state or force_refresh) and clients_connected:
                    send_notification = True

                if not send_notification and clients_connected:
                    if now < self._ts_delay_until:
                        Log.warn(f"delay update state until {self._ts_delay_until - now:.2f} sec")
                    if now > self._ts_delay_until and now - ts_state_notified > self._check_delay * 2.0:
                        # check own state
                        count_topics = len(nmd.ros_node.get_topic_names_and_types())
                        if self._count_topics != count_topics:
                            Log.debug(f"count_topics old/new: {self._count_topics} / {count_topics}")
                            send_notification = True
                            self._count_topics = count_topics
                        else:
                            count_services = len(nmd.ros_node.get_service_names_and_types())
                            if self._count_services != count_services:
                                Log.debug(f"count_services old/new: {self._count_services} / {count_services}")
                                send_notification = True
                                self._count_services = count_services
                            else:
                                unique_nodes = set()
                                for name, ns in nmd.ros_node.get_node_names_and_namespaces():
                                    unique_nodes.add(os.path.join(ns, name))
                                count_nodes = len(unique_nodes)
                                if self._count_nodes != count_nodes:
                                    Log.debug(f"count_nodes old/new: {self._count_nodes} / {count_nodes}")
                                    send_notification = True
                                    self._count_nodes = count_nodes

                if send_notification:
                    try:
                        # trigger screen servicer to update
                        nmd.launcher.server.screen_servicer.system_change()
                        # trigger the update of the ros state
                        # The updates are received by callback defined in the __init__()
                        self._state_jsonify.update_state(force_refresh)
                        with self._ros_node_state_mutex:
                            self._force_refresh = False
                    except Exception:
                        msg = traceback.format_exc()
                        Log.warn(msg)
                        warnings_group: SystemWarningGroup = SystemWarningGroup(SystemWarningGroup.ID_ROS_STATE)
                        warnings_group.append(SystemWarning(msg=msg))
                        if self.monitor_servicer is not None:
                            self.monitor_servicer.update_warning_groups([warnings_group])

                # check time jumps
                now = time.time()
                if now < self._timestamp:
                    time_jump_msg = "Time jump into past detected! Restart all ROS nodes, includes MAS nodes, please!"
                    Log.warn(time_jump_msg)
                    w_time_jump = SystemWarningGroup(SystemWarningGroup.ID_TIME_JUMP)
                    w_time_jump.append(SystemWarning(msg='Timejump into past detected!',
                                                     hint='Restart all ROS nodes, includes master_discovery, please! master_discovery shutting down in 5 seconds!'))
                    if self.monitor_servicer is not None:
                        self.monitor_servicer.update_warning_groups([w_time_jump])
                # store the current time in any case, otherwise the warning would be repeated forever
                self._timestamp = now
                # check for timeouted provider
                removed_uris = []
                endpoints_copy = None
                with self._lock_check:
                    for uri, ts in self._endpoints_ts.items():
                        if now - ts > self._endpoint_timeout_sec:
                            Log.info(f"{self.__class__.__name__}: remove outdated daemon {uri}, not seen for {now - ts} sec")
                            if uri in self._endpoints:
                                del self._endpoints[uri]
                            removed_uris.append(uri)
                    for uri in removed_uris:
                        del self._endpoints_ts[uri]
                    if len(removed_uris) > 0:
                        endpoints_copy = dict(self._endpoints)
                if endpoints_copy is not None:
                    # publish outside of the lock, the name resolution can block
                    self._publish_masters(endpoints_copy)
            except Exception:
                Log.warn(traceback.format_exc())
            # wait interruptible, so stop() does not have to wait for the full delay
            if self._shutdown_event.wait(self._check_delay):
                break

    def stop(self):
        '''
        Unregister the subscribed topic
        '''
        self._on_shutdown = True
        self._shutdown_event.set()
        self._state_jsonify.stop()
        for attribute in ('sub_discovered_state', 'sub_endpoints', 'sub_participants'):
            subscription = getattr(self, attribute, None)
            if subscription is not None:
                try:
                    nmd.ros_node.destroy_subscription(subscription)
                except Exception:
                    Log.debug(f"{self.__class__.__name__}: error while destroy {attribute}: {traceback.format_exc()}")
                delattr(self, attribute)
        if self._thread_check_discovery_node is not None:
            self._thread_check_discovery_node.join(timeout=2.0)
            self._thread_check_discovery_node = None
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
        endpoints_copy = None
        with self._lock_check:
            if msg.on_shutdown:
                if msg.uri in self._endpoints:
                    is_new = True
                    Log.info(f"{self.__class__.__name__}: remove daemon {msg.uri}")
                    del self._endpoints[msg.uri]
                    endpoints_copy = dict(self._endpoints)
            else:
                if msg.uri in self._endpoints:
                    other = self._endpoints[msg.uri]
                    is_new = msg.name != other.name
                    is_new |= msg.ros_name != other.ros_name
                    is_new |= msg.ros_domain_id != other.ros_domain_id
                    is_new |= msg.pid != other.pid
                else:
                    is_new = True
                if is_new:
                    self._endpoints[msg.uri] = msg
                    endpoints_copy = dict(self._endpoints)
                self._endpoints_ts[msg.uri] = time.time()
        if endpoints_copy is not None:
            # publish outside of the lock, the name resolution can block the ros callback
            self._publish_masters(endpoints_copy)

    def get_provider_list(self) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.provider.get_list]")
        with self._lock_check:
            endpoints_copy = dict(self._endpoints)
        return json.dumps(self._endpoints_to_provider(endpoints_copy), cls=SelfEncoder)

    def get_node_list(self, forceRefresh: bool = False) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.nodes.get_list]; forceRefresh: {forceRefresh}")
        with self._ros_node_state_mutex:
            self._get_ros_node_list(forceRefresh)
            return self._ros_node_list_str

    def _topic_in_filter(self, topic_name: str, topic_type: str, filter: List[RosTopicId]) -> bool:
        if not filter:
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

    def get_service_list(self, filter: List[RosTopicId] = None) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.services.get_list]")
        with self._ros_service_state_mutex:
            if not filter:
                return self._ros_service_list_str
            filtered: List[RosService] = []
            for id, service in self._ros_service_dict.items():
                if self._topic_in_filter(id[0], id[1], filter):
                    filtered.append(service)
        return json.dumps(filtered, cls=SelfEncoder)

    def get_topic_list(self, filter: List[RosTopicId] = None) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.topics.get_list]")
        with self._ros_topic_state_mutex:
            if not filter:
                return self._ros_topic_list_str
            filtered: List[RosTopic] = []
            for id, topic in self._ros_topic_dict.items():
                if self._topic_in_filter(id[0], id[1], filter):
                    filtered.append(topic)
        return json.dumps(filtered, cls=SelfEncoder)

    def get_loggers(self, name: str, loggers: List[str] = None) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.nodes.get_loggers] for '{name}', loggers: {loggers}")
        if not HAS_LOGGER_INTERFACE:
            raise Exception("ros2 version on this client does not support logger interface!")
        requested_loggers: List[str] = list(loggers) if loggers else []
        logger_names = list(requested_loggers) if requested_loggers else [name.replace("/", ".").strip("."), "rcl"]
        loggerConfigs: List[LoggerConfig] = []
        # get logger names if loggers list is empty
        if not requested_loggers:
            service_name = '%s/logger_list' % name
            try:
                service_available = False
                with self._ros_service_state_mutex:
                    service_available = service_name in self._ros_service_name_set
                if service_available:
                    Log.debug(f"{self.__class__.__name__}: call service '{service_name}'")
                    request_list = GetLoggerLevels.Request()
                    request_list.names = []
                    get_logger = nmd.launcher.call_service(
                        service_name, GetLoggerLevels, request_list, callback_group=self._callback_group_logger)
                    if get_logger:
                        for logger in get_logger.levels:
                            logger_names.append(logger.name)
            except Exception as e:
                Log.warn(f"{self.__class__.__name__}: failed call service '{service_name}': {e}")
        # get current logger levels
        service_name = '%s/get_logger_levels' % name
        with self._ros_service_state_mutex:
            if service_name not in self._ros_service_name_set:
                raise Exception(f"logger service '{service_name}' not found")
        Log.debug(f"{self.__class__.__name__}: call service '{service_name}'")
        request_list = GetLoggerLevels.Request()
        request_list.names = logger_names
        get_logger = nmd.launcher.call_service(service_name, GetLoggerLevels,
                                               request_list, timeout_sec=5.0, callback_group=self._callback_group_logger)
        if get_logger:
            Log.debug(f"{self.__class__.__name__}: found {len(get_logger.levels)} logger levels on '{service_name}'")
            for logger in get_logger.levels:
                loggerConfigs.append(LoggerConfig(
                    level=LoggerConfig.LogLevelType.fromRos2(logger.level), name=logger.name))
        else:
            raise Exception(f"failed logger service call '{service_name}'; response: {get_logger}")
        return json.dumps(loggerConfigs, cls=SelfEncoder)

    def set_logger_level(self, name: str, loggers: List[LoggerConfig]) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.nodes.set_logger_level] for '{name}'")
        if not HAS_LOGGER_INTERFACE:
            raise Exception("ros2 version on this client does not support logger interface!")
        # request the current logger
        service_name_get = '%s/set_logger_levels' % name
        with self._ros_service_state_mutex:
            if service_name_get not in self._ros_service_name_set:
                raise Exception(f"logger service '{service_name_get}' not found")
        request_set = SetLoggerLevels.Request()
        for logger in loggers:
            log_level = LoggerLevel()
            log_level.name = logger.name
            log_level.level = LoggerConfig.LogLevelType.toRos2(logger.level)
            request_set.levels.append(log_level)
        set_logger = nmd.launcher.call_service(service_name_get, SetLoggerLevels,
                                               request_set, callback_group=self._callback_group_logger)
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

    def stop_node(self, name: str) -> str:
        Log.info(f"{self.__class__.__name__}: Request to stop node '{name}'")
        node: RosNode = self.get_ros_node(name)
        if node is None:
            node = self.get_ros_node_by_id(name)
        if node is None:
            return json.dumps({'result': False, 'message': f'{name} not found'}, cls=SelfEncoder)
        container_name = node.container_name
        if not container_name:
            # composable node are determine by service call.
            # we have to search in composable state for the node
            with self._ros_composable_mutex:
                for state in self._composables_nodes:
                    if node.name in state.nodes:
                        container_name = state.containerName
                        node.container_name = container_name
                        break
        unloaded = False
        if container_name:
            unloaded = self.stop_composed_node(node, container_name)
        if unloaded:
            result = json.dumps({'result': True, 'message': ''}, cls=SelfEncoder)
        else:
            # it was not a composable node -> try to stop
            result = nmd.launcher.server.screen_servicer.kill_node(node.name, signal.SIGTERM)
        nmd.launcher.server.screen_servicer.system_change()
        nmd.launcher.server.launch_servicer.node_stopped(node.name)
        return result

    def _publisher_node_name(self, topic_name: str) -> str:
        '''
        Creates the node name used by the mas publisher for the given topic.
        '''
        ns, name = ros2_publisher_nodename_tuple(topic_name)
        fullname = f"{ns}/{name}".strip('/').replace('/', '_')
        return f"/{fullname}"

    def _join_node_name(self, ns: str, name: str) -> str:
        '''
        Joins namespace and node name to a fully qualified ROS name.
        os.path.join() can not be used here, because absolute names would discard the namespace.
        '''
        return f"/{ns.strip('/')}/{name.strip('/')}".replace('//', '/')

    def has_publisher(self, topic_name: str) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.publisher.has]: {str(topic_name)}")
        fullname = self._publisher_node_name(topic_name)
        node: RosNode = self.get_ros_node(fullname)
        result = node is not None
        return json.dumps({"result": result, "message": ""}, cls=SelfEncoder)

    def stop_publisher(self, topic_name: str) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.publisher.stop]: {str(topic_name)}")
        fullname = self._publisher_node_name(topic_name)
        # stop_node() returns already a json encoded result
        return self.stop_node(fullname)

    def stop_subscriber(self, topic_name: str) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.subscriber.stop]: {str(topic_name)}")
        ns, name = ros2_subscriber_nodename_tuple(topic_name)
        # stop_node() returns already a json encoded result
        return self.stop_node(self._join_node_name(ns, name))

    def stop_action(self, action_name: str) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.action.stop]: {str(action_name)}")
        ns, name = ros2_action_nodename_tuple(action_name)
        # stop_node() returns already a json encoded result
        return self.stop_node(self._join_node_name(ns, name))

    def stop_action_introspection(self, action_name: str) -> str:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.action.introspection.stop]: {str(action_name)}"
        )
        ns, name = ros2_action_introspection_nodename_tuple(f"{action_name}")
        # stop_node() returns already a json encoded result
        return self.stop_node(self._join_node_name(ns, name))

    def stop_service_introspection(self, service_name: str) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.service.introspection.stop]: {service_name}")
        ns, name = ros2_service_introspection_nodename_tuple(service_name)
        # stop_node() returns already a json encoded result
        return self.stop_node(self._join_node_name(ns, name))

    def get_provider_timestamp(self, timestamp) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.provider.get_timestamp], timestamp: {timestamp}")
        now_ms = time.time() * 1000
        return json.dumps({'timestamp': now_ms, "diff": now_ms - float(timestamp)}, cls=SelfEncoder)

    def stop_composed_node(self, node: RosNode, container_name: str = None) -> bool:
        # try to unload node from container
        if container_name is None:
            container_name = node.container_name
        Log.info(f"{self.__class__.__name__}: -> unload '{node.name}' from '{container_name}'")
        # TODO: shutdown lifecycle nodes before unload
        try:
            container_node: RosNode = self.get_ros_node(container_name)
            if container_node is not None:
                unique_id_in_container = self.get_composed_node_id(container_name, node.name)
                if unique_id_in_container > -1:
                    service_unload_node = f'{container_node.name}/_container/unload_node'
                    Log.info(
                        f"{self.__class__.__name__}: -> unload '{node.name}' with id '{unique_id_in_container}' using service '{service_unload_node}'")
                    request = UnloadNode.Request()
                    request.unique_id = unique_id_in_container
                    response = nmd.launcher.call_service(
                        service_unload_node, UnloadNode, request, callback_group=self._callback_group_composed, timeout_sec=1.0)
                    if hasattr(response, "success") and response.success:
                        return True
                    elif not hasattr(response, "success"):
                        Log.warn(f"{self.__class__.__name__}: -> unload '{node.name}' error while call unload_node service")
                    else:
                        Log.warn(f"{self.__class__.__name__}: -> unload '{node.name}' error '{response.error_message}'")
                return False
            else:
                Log.warn(f"{self.__class__.__name__}: -> Container node '{container_name}' not found")
        except Exception as err:
            Log.warn(f"{err}")
        return False

    def get_composed_node_id(self, container_name: str, node_name: str) -> Number:
        # Normally, you would call the _container/list_nodes service to get the ID of the composable node.
        # However, this can take a long time if the service is unavailable but many composable nodes need to be shut down.
        # Therefore, we fall back to the last known status.
        with self._ros_composable_mutex:
            for composable in self._composables_nodes:
                if composable.containerName == container_name:
                    for name, unique_id in composable.composableIds:
                        if name == node_name:
                            return unique_id
        # service_list_nodes = f'{container_name}/_container/list_nodes'
        # Log.debug(f"{self.__class__.__name__}: list nodes from '{service_list_nodes}'")
        # request_list = ListNodes.Request()
        # response_list = nmd.launcher.call_service(
        #     service_list_nodes, ListNodes, request_list, callback_group=self._callback_group_composed, timeout_sec=1.0)
        # if response_list is not None:
        #     for name, unique_id in zip(response_list.full_node_names, response_list.unique_ids):
        #         if name == node_name:
        #             return unique_id
        return -1

    def _get_ros_node_list(self, forceRefresh: bool = False) -> List[RosNode]:
        # the status is updated in _check_discovery_node() in a thread
        # in the meantime, the cached list is returned
        # after the state is ready, a 'ros.nodes.changed' notification will be send
        with self._ros_node_state_mutex:
            if forceRefresh:
                self._force_refresh = True
            if (self._ros_node_list is None or forceRefresh):
                self._ros_node_list = []
                self._ros_node_list_str = json.dumps(self._ros_node_list, cls=SelfEncoder)
                with self._ros_service_state_mutex:
                    self._ros_service_dict = {}
                with self._ros_topic_state_mutex:
                    self._ros_topic_dict = {}
            return self._ros_node_list

    def get_ros_node(self, node_name: str) -> Union[RosNode, None]:
        with self._ros_node_state_mutex:
            node_list: List[RosNode] = self._get_ros_node_list()
            for node in node_list:
                if node_name == node.name:
                    return node
            return None

    def get_ros_node_by_id(self, node_id: str) -> Union[RosNode, None]:
        with self._ros_node_state_mutex:
            node_list: List[RosNode] = self._get_ros_node_list()
            for node in node_list:
                if node_id == node.id:
                    return node
            return None

    def delay_update_state(self, delay_msg: DelayRosUpdateState) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.delay_update_state]: {delay_msg.sec} sec")
        if delay_msg.sec <= 0:
            return
        now = time.time()
        self._ts_delay_until = now + delay_msg.sec
