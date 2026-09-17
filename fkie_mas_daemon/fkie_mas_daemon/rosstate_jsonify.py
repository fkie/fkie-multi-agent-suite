# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from __future__ import annotations

import os
import queue
import threading
import traceback
from collections.abc import Callable, Sequence
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from typing import Any, Optional

import psutil
from composition_interfaces.srv import ListNodes
from fkie_mas_pylib.interface.runtime_interface import (
    EndpointInfo,
    IncompatibleQos,
    LifecycleTransition,
    RosComposable,
    RosLifecycleState,
    RosNode,
    RosQos,
    RosService,
    RosTopic,
    SystemWarning,
    SystemWarningGroup,
)
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.service.future import WaitFuture, create_service_future, wait_until_futures_done
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.system.host import get_local_addresses
from rclpy.callback_groups import ReentrantCallbackGroup  # MutuallyExclusiveCallbackGroup
from rclpy.node import Subscription
from rclpy.qos import QoSCompatibility, QoSProfile, qos_check_compatible

import fkie_mas_daemon as nmd
from fkie_mas_daemon.monitor_servicer import MonitorServicer
from fkie_mas_msgs.msg import ParticipantEntitiesInfo, Participants
from fkie_mas_pylib import names

LIFECYCLE_AVAILABLE = False
try:
    from lifecycle_msgs.msg import TransitionEvent
    from lifecycle_msgs.srv import GetAvailableTransitions, GetState

    LIFECYCLE_AVAILABLE = True
except ImportError:
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
IsRequest = bool

# (topic name without prefix, message/service type)
TopicKey = tuple[str, str]
# hashable representation of a QoS profile, used to cache compatibility checks
QosKey = Optional[tuple]

# DDS topic name prefixes used for services (request/reply/status)
DDS_SERVICE_PREFIXES = ("rr/", "rq/", "rs/")
DDS_TOPIC_PREFIX = "rt/"
# reload the local network addresses every N update cycles (network may change)
LOCAL_ADDRESS_REFRESH_CYCLES = 30
# upper bound for pending lifecycle events; events are batched, so dropping
# single events during a transition storm is not critical
LIFECYCLE_QUEUE_MAX_SIZE = 1000
# maximum number of attempts to query the composable nodes of a container
COMPOSABLE_RETRY_MAX = 3
# timeout in seconds for the service calls
SERVICE_CALL_TIMEOUT = 10
# warnings about topics used with more than one message type are reported in an
# own group, so that they are not overwritten by lifecycle/composable warnings
WARNING_GROUP_TOPIC_TYPES = f"{SystemWarningGroup.ID_ROS_STATE}_TOPIC_TYPES"


class QosPub:
    node: RosNode
    qos_profile: Any
    # cached hashable key of the qos_profile, None if it could not be created
    qos_key: QosKey

    def __init__(self, node: RosNode, qos_profile: Any, qos_key: QosKey = None):
        self.node = node
        self.qos_profile = qos_profile
        self.qos_key = qos_key


class CachedData:
    node_dict: dict[tuple[NodeNamespace, NodeName, ParticipantGid], RosNode]
    topic_objs: dict[tuple[TopicNameWoPrefix, TopicType], RosTopic]
    service_objs: dict[tuple[ServiceNameWoPrefix, ServiceType], RosService]
    # node full name -> list of screen session names (inverted screen list)
    screens_by_node: dict[NodeFullName, list[str]]
    # (namespace, basename) -> pids of all processes started with '__node:='.
    # Created once per update cycle to avoid repeated (expensive) process scans.
    processes: dict[tuple[str, str], list[int]]
    # snapshot of all node names known to run inside a composable container
    composable_node_names: set[NodeFullName]
    # collected local node names of the current update cycle
    local_node_names: list[NodeFullName]
    local_node_name_set: set[NodeFullName]
    # O(1) index sets to avoid quadratic membership tests in lists
    node_publisher_ids: dict[NodeId, set[TopicKey]]
    node_subscriber_ids: dict[NodeId, set[TopicKey]]
    topic_publisher_nodes: dict[TopicKey, set[NodeId]]
    topic_subscriber_nodes: dict[TopicKey, set[NodeId]]
    service_provider_nodes: dict[TopicKey, set[NodeId]]
    node_service_ids: dict[NodeId, set[TopicKey]]
    # cache for QoS compatibility results: (publisher qos key, subscriber qos key)
    qos_compatibility: dict[tuple[tuple, tuple], tuple[QoSCompatibility, str]]
    # topic names which are used with more than one message type
    type_mismatch: dict[TopicNameWoPrefix, list[TopicType]]

    def __init__(self):
        self.node_dict = {}
        self.topic_objs = {}
        self.service_objs = {}
        self.screens_by_node = {}
        self.processes = {}
        self.composable_node_names = set()
        self.local_node_names = []
        self.local_node_name_set = set()
        self.node_publisher_ids = {}
        self.node_subscriber_ids = {}
        self.topic_publisher_nodes = {}
        self.topic_subscriber_nodes = {}
        self.service_provider_nodes = {}
        self.node_service_ids = {}
        self.qos_compatibility = {}
        self.type_mismatch = {}


class RosStateJsonify:
    def __init__(
        self,
        *,
        cb_nodes: Callable[[list[RosNode]], None],
        cb_topics: Callable[[dict[tuple[TopicNameWoPrefix, TopicType], RosTopic]], None],
        cb_services: Callable[[dict[tuple[ServiceNameWoPrefix, ServiceType], RosService]], None],
        cb_composables: Callable[[list[RosComposable]], None],
        cb_lifecycle: Callable[[list[RosLifecycleState]], None],
        monitor_servicer: MonitorServicer | None = None,
    ):
        Log.debug("Create RosStateJsonify")
        self._callback_group = ReentrantCallbackGroup()
        self._shutdown = False
        self._cb_nodes = cb_nodes
        self._cb_topics = cb_topics
        self._cb_services = cb_services
        self._cb_composables = cb_composables
        self._cb_lifecycle = cb_lifecycle
        self.monitor_servicer = monitor_servicer
        self._local_node_names: list[NodeFullName] = []
        self._composable_nodes: dict[NodeId, RosComposable] = {}
        self._lock = threading.RLock()
        self._lifecycle_subscriptions: dict[NodeId, Subscription] = {}
        self._local_addresses = get_local_addresses()
        self._update_cycles = 0
        self._participant_infos: dict[ParticipantGid, ParticipantEntitiesInfo] = {}
        self._ros_service_dict: dict[tuple[ServiceNameWoPrefix, ServiceType], RosService] = {}
        self._ros_topic_dict: dict[tuple[TopicNameWoPrefix, TopicType], RosTopic] = {}
        self._use_name_as_node_id = self.get_rwm_implementation() in ["rmw_zenoh_cpp"]

        # Tracks unassigned local nodes (no process, no screen) that are not yet
        # listed in any known composable container. Used to detect new composable
        # nodes between runs without triggering redundant updates.
        self._unassigned_composable_nodes: set[NodeFullName] = set()
        # containers with a running (asynchronous) list_nodes request; they must
        # not be removed by the cleanup of the update cycle
        self._pending_composable_ids: set[NodeId] = set()
        # incremented on forceRefresh; results of older requests are discarded
        self._state_generation = 0

        # Lifecycle transition events are handled by a single worker thread.
        # This avoids spawning one thread per received event.
        self._lifecycle_event_queue: queue.Queue = queue.Queue(maxsize=LIFECYCLE_QUEUE_MAX_SIZE)
        self._lifecycle_worker: threading.Thread | None = None
        # shared executor for all asynchronous service calls; avoids piling up
        # threads if containers answer slowly
        self._executor = ThreadPoolExecutor(max_workers=3, thread_name_prefix="ros_state_jsonify")

    def stop(self):
        """Stops all background work and destroys created subscriptions."""
        self._shutdown = True
        # wake up the lifecycle worker so that it can terminate
        try:
            self._lifecycle_event_queue.put_nowait(None)
        except Exception:
            pass
        worker = self._lifecycle_worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        try:
            self._executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")
        with self._lock:
            for _node_id, sub in self._lifecycle_subscriptions.items():
                try:
                    Log.debug(f"{self.__class__.__name__}: unregister subscription {sub.topic_name}")
                    if nmd.ros_node is not None:
                        nmd.ros_node.destroy_subscription(sub)
                except Exception:
                    Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")
            self._lifecycle_subscriptions.clear()
            self._pending_composable_ids.clear()

    def get_rwm_implementation(self) -> str:
        # 'rmw_fastrtps_cpp' is the default implementation for the currently supported distributions (e.g. jazzy)
        result = os.environ.get("RMW_IMPLEMENTATION", "")
        if not result:
            result = "rmw_fastrtps_cpp"
        return result

    def _submit(self, func: Callable, *args) -> None:
        """Runs a task in the shared executor, ignored after shutdown."""
        if self._shutdown:
            return
        try:
            self._executor.submit(func, *args)
        except RuntimeError:
            # executor already shut down
            Log.debug(f"{self.__class__.__name__}: executor rejected task, shutdown in progress")

    # ------------------------------------------------------------------ #
    #  lifecycle handling
    # ------------------------------------------------------------------ #

    def _on_lifecycle_event(self, _msg: TransitionEvent, node_id: NodeId, node_name: NodeFullName):
        self._queue_lifecycle_update(node_id, node_name)

    def _queue_lifecycle_update(self, node_id: NodeId, node_name: NodeFullName) -> None:
        """Adds a node to the lifecycle update queue and starts the worker if needed."""
        if self._shutdown:
            return
        with self._lock:
            if self._lifecycle_worker is None or not self._lifecycle_worker.is_alive():
                self._lifecycle_worker = threading.Thread(target=self._thread_lifecycle_worker, daemon=True)
                self._lifecycle_worker.start()
        try:
            self._lifecycle_event_queue.put_nowait((node_id, node_name))
        except queue.Full:
            # the pending batch will query the current state anyway
            Log.debug(f"{self.__class__.__name__}: lifecycle queue full, dropped event of {node_name}")

    def _thread_lifecycle_worker(self) -> None:
        """Handles lifecycle transition events. Events received while an update is
        running are collected and merged into a single service call round."""
        while not self._shutdown:
            try:
                item = self._lifecycle_event_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            if item is None:
                # shutdown requested
                break
            pending: dict[NodeId, NodeFullName] = {item[0]: item[1]}
            # batch all events which are already in the queue
            stop_requested = False
            while True:
                try:
                    next_item = self._lifecycle_event_queue.get_nowait()
                except queue.Empty:
                    break
                if next_item is None:
                    stop_requested = True
                    break
                pending[next_item[0]] = next_item[1]
            if pending and not self._shutdown:
                self._thread_update_lifecycle_call(
                    [RosNode(node_id, node_name) for node_id, node_name in pending.items()]
                )
            if stop_requested:
                break

    def _subscribe_lifecycle(self, *, topic_name: str, node_id: NodeId, node_name: NodeFullName) -> bool:
        """Creates a transition event subscription. Must be called with the lock held."""
        if LIFECYCLE_AVAILABLE and node_id not in self._lifecycle_subscriptions:
            Log.debug(f"{self.__class__.__name__}: subscribe to {topic_name}")
            sub = nmd.ros_node.create_subscription(
                TransitionEvent, topic_name, partial(self._on_lifecycle_event, node_id=node_id, node_name=node_name), 1
            )
            self._lifecycle_subscriptions[node_id] = sub
            return True
        return False

    # ------------------------------------------------------------------ #
    #  name/type conversion
    # ------------------------------------------------------------------ #

    @classmethod
    def get_message_type(cls, dds_type: str) -> str:
        result = dds_type
        if result:
            result = result.replace("::", "/")
            result = result.replace("/dds_", "")
            # NOTE: the trailing type suffix behind the last '/' is currently not removed:
            #   last_slash_index = result.rfind('/')
            #   if last_slash_index != -1:
            #       underscore_index = result.rfind('_', last_slash_index)
            #       if underscore_index != -1:
            #           result = result[:underscore_index]
            result = result.rstrip("_")
        return result

    @classmethod
    def get_service_type(cls, dds_service_type: str) -> str:
        result = dds_service_type
        for suffix in ["_Response_", "_Request_"]:
            if result.endswith(suffix):
                # keep the leading '_' of the suffix, it is removed by the
                # rstrip('_') in get_message_type() together with the type suffix
                result = result[: -len(suffix) + 1]
                break
        return cls.get_message_type(result)

    @classmethod
    def get_service_name(cls, dds_service_name: str) -> str:
        result = dds_service_name
        for suffix in ["Reply", "Request"]:
            if result.endswith(suffix):
                result = result[: -len(suffix)]
                break
        return result

    @classmethod
    def _node_full_name(cls, node_ns: str, node_name: str) -> NodeFullName:
        """Joins namespace and node name with a ROS separator (not os.path.join)."""
        return f"{node_ns.rstrip('/')}/{node_name}"

    @classmethod
    def _guid_arr_to_str(cls, gid: Sequence[int]) -> str:
        try:
            return bytes(gid).hex(".").upper()
        except Exception:
            return ".".join(f"{c:02X}" for c in gid)

    def _guid_to_str(self, guid: Any) -> ParticipantGid:
        data = guid.data.tolist() if hasattr(guid.data, "tolist") else list(guid.data)
        return self._guid_arr_to_str(data[0:12])

    def parse_node_name(self, node_name: str) -> tuple[NodeName, NodeNamespace]:
        full_node_name = node_name
        if not full_node_name.startswith("/"):
            full_node_name = "/" + full_node_name
        namespace, node_basename = full_node_name.rsplit("/", 1)
        if namespace == "":
            namespace = "/"
        return node_basename, namespace

    # ------------------------------------------------------------------ #
    #  process/screen/location helper
    # ------------------------------------------------------------------ #

    def _scan_processes(self) -> dict[tuple[str, str], list[int]]:
        """Scans all processes once and returns an index (namespace, basename) -> [pid]
        for all processes started with a '__node:=' argument."""
        index: dict[tuple[str, str], list[int]] = {}
        for process in psutil.process_iter(["pid", "cmdline"]):
            try:
                cmdline = process.info["cmdline"]
                if not cmdline:
                    continue
                node_name: str | None = None
                node_ns = ""
                for arg in cmdline:
                    if arg.startswith("__node:="):
                        node_name = arg[len("__node:=") :]
                    elif arg.startswith("__ns:="):
                        node_ns = arg[len("__ns:=") :].rstrip("/")
                if node_name:
                    index.setdefault((node_ns, node_name), []).append(process.info["pid"])
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
            except Exception:
                Log.debug(
                    f"{self.__class__.__name__}: ignored exception while scanning processes: {traceback.format_exc()}"
                )
        return index

    def _index_screens(self) -> dict[NodeFullName, list[str]]:
        """Builds node full name -> [screen session names] once per update cycle."""
        screens_by_node: dict[NodeFullName, list[str]] = {}
        try:
            for session_name, screen_node_name in screen.get_active_screens().items():
                screens_by_node.setdefault(screen_node_name, []).append(session_name)
        except Exception:
            Log.debug(f"{self.__class__.__name__}: ignored exception while reading screens: {traceback.format_exc()}")
        return screens_by_node

    def find_node(self, name: NodeFullName, processes: dict[tuple[str, str], list[int]] | None = None) -> list[int]:
        """Returns the pids of all processes started with __node:=<basename> [__ns:=<namespace>]."""
        if processes is None:
            processes = self._scan_processes()
        # a node without explicit __ns:= runs in the root namespace
        return list(processes.get((names.namespace(name).rstrip("/"), names.basename(name)), []))

    # kept for API compatibility
    findNode = find_node

    def is_location_local(self, location: str) -> bool:
        if "SHM" in location:
            return True
        if "127.0." in location:
            return True
        for loc_addr in self._local_addresses:
            if loc_addr in location:
                return True
        return False

    # ------------------------------------------------------------------ #
    #  public accessors
    # ------------------------------------------------------------------ #

    def get_services(self) -> dict[tuple[ServiceNameWoPrefix, ServiceType], RosService]:
        return self._ros_service_dict

    def get_topics(self) -> dict[tuple[TopicNameWoPrefix, TopicType], RosTopic]:
        return self._ros_topic_dict

    def get_local_node_names(self) -> list[NodeFullName]:
        return self._local_node_names

    def apply_participants(self, msg: Participants):
        # update the participant info (IP addresses)
        new_ros_state: dict[ParticipantGid, ParticipantEntitiesInfo] = {}
        for participant in msg.participants:
            new_ros_state[self._guid_to_str(participant.guid)] = participant
        with self._lock:
            self._participant_infos = new_ros_state

    def update_all_composables(self) -> None:
        """Updates all composable nodes currently tracked in self._composable_nodes."""
        with self._lock:
            nodes_to_update = [
                RosNode(node_id, composable.containerName) for node_id, composable in self._composable_nodes.items()
            ]
            self._pending_composable_ids.update(n.id for n in nodes_to_update)
            generation = self._state_generation
        if nodes_to_update:
            self._submit(self._thread_update_composables_call, nodes_to_update, 1, generation)

    # ------------------------------------------------------------------ #
    #  state update
    # ------------------------------------------------------------------ #

    def _has_new_unassigned_nodes(self, discovered_nodes: list[RosNode]) -> list[NodeId]:
        """Checks if new unassigned local nodes appeared since the last run.

        Returns a list of container NodeIds that need to be refreshed. Only
        triggers if genuinely new nodes were discovered compared to the previous
        invocation.
        """
        containers_to_update: list[NodeId] = []
        current_unassigned: set[NodeFullName] = set()

        with self._lock:
            # collect all node names already known to belong to a container
            known_composable_names: set[NodeFullName] = set()
            for composable in self._composable_nodes.values():
                known_composable_names.update(composable.nodes)

            # identify local nodes without own process/screen that are not
            # assigned to any composable container
            for node in discovered_nodes:
                if node.is_local and not node.process_ids and not node.screens:
                    if node.name not in known_composable_names and "_impl_" not in node.name:
                        current_unassigned.add(node.name)

            # only trigger refresh if there are nodes that were NOT unassigned
            # in the previous run (i.e. genuinely new)
            if current_unassigned - self._unassigned_composable_nodes:
                containers_to_update = list(self._composable_nodes.keys())

            # store current state for the next comparison
            self._unassigned_composable_nodes = current_unassigned

        return containers_to_update

    def update_state(self, forceRefresh: bool) -> None:
        """Creates the ROS graph state and reports it through the registered callbacks."""
        Log.debug(f"{self.__class__.__name__}: create graph for websocket")
        if self._shutdown or nmd.ros_node is None:
            Log.debug(f"{self.__class__.__name__}: no ros node available, skip update")
            return
        if forceRefresh:
            with self._lock:
                self._composable_nodes = {}
                self._unassigned_composable_nodes = set()
                self._pending_composable_ids.clear()
                # discard results of requests started before the refresh
                self._state_generation += 1
                if self.monitor_servicer:
                    self.monitor_servicer.remove_warning_group(SystemWarningGroup.ID_ROS_STATE)

        self._update_cycles += 1
        if self._update_cycles % LOCAL_ADDRESS_REFRESH_CYCLES == 0:
            # the network configuration may have changed in the meantime
            self._local_addresses = get_local_addresses()

        cached_data: CachedData = CachedData()
        cached_data.screens_by_node = self._index_screens()
        # scan the processes only once per update cycle
        cached_data.processes = self._scan_processes()
        with self._lock:
            for composable in self._composable_nodes.values():
                cached_data.composable_node_names.update(composable.nodes)

        result: list[RosNode] = []
        transition_event_publisher: list[NodeId] = []
        new_transition_event_nodes: list[RosNode] = []
        found_composable_nodes: list[NodeId] = []
        new_composable_nodes: list[RosNode] = []

        topic_list = nmd.ros_node.get_topic_names_and_types(no_demangle=True)
        for topic_name, _topic_types in topic_list:
            # a topic name is handled only once, all types are reported by the endpoint infos
            is_request = self._is_request_topic(topic_name)
            # publisher QoS profiles grouped by topic type; endpoints with
            # different types never match, so they must not be compared
            pub_qos_by_type: dict[TopicType, list[QosPub]] = {}
            sub_types: set[TopicType] = set()

            pub_infos = nmd.ros_node.get_publishers_info_by_topic(topic_name, True)
            for pub_info in pub_infos:
                if "_NODE_NAME_UNKNOWN_" in pub_info.node_name or "_NODE_NAMESPACE_UNKNOWN_" in pub_info.node_namespace:
                    continue
                try:
                    if self._use_name_as_node_id:
                        gid = self._node_full_name(pub_info.node_namespace, pub_info.node_name)
                    else:
                        gid = self._guid_arr_to_str(pub_info.endpoint_gid[0:12])
                    t_gid = self._guid_arr_to_str(pub_info.endpoint_gid)
                    ros_node, is_new_node = self._get_node_from(
                        pub_info.node_namespace, pub_info.node_name, gid, cached_data
                    )
                except Exception:
                    Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")
                    continue
                # register the node before processing the endpoint, so that it is
                # reported even if the topic handling below fails
                if is_new_node:
                    result.append(ros_node)
                try:
                    tp = self._get_topic_from(topic_name, pub_info.topic_type, cached_data)
                    # topic or service ?
                    if isinstance(tp, RosTopic):
                        Log.debug(
                            f"{self.__class__.__name__}:      add publisher {ros_node.id} "
                            f"{pub_info.node_namespace}/{pub_info.node_name} for {tp.name}"
                        )
                        topic_key: TopicKey = (tp.name, tp.msg_type)
                        publisher_nodes = cached_data.topic_publisher_nodes.setdefault(topic_key, set())
                        if ros_node.id not in publisher_nodes:
                            endpoint_info = EndpointInfo(t_gid, ros_node.id, self._get_qos(pub_info.qos_profile), [])
                            tp.publisher.append(endpoint_info)
                            publisher_nodes.add(ros_node.id)
                        node_publishers = cached_data.node_publisher_ids.setdefault(ros_node.id, set())
                        if topic_key not in node_publishers:
                            ros_node.publishers.append(tp.get_topic_id())
                            node_publishers.add(topic_key)
                        pub_qos_by_type.setdefault(pub_info.topic_type, []).append(
                            QosPub(ros_node, pub_info.qos_profile, self._qos_cache_key(pub_info.qos_profile))
                        )
                        # the node is a system node if topic type is from MAS messages
                        discover_state_publisher = "fkie_mas_msgs::msg::dds_::DiscoveredState_" in pub_info.topic_type
                        endpoint_publisher = "fkie_mas_msgs::msg::dds_::Endpoint_" in pub_info.topic_type
                        ros_node.system_node |= discover_state_publisher or endpoint_publisher
                        # check for lifecycle transition event topic
                        if self._is_local_lifecycle_transition_topic(tp.name, tp.msg_type, ros_node, cached_data):
                            with self._lock:
                                if self._subscribe_lifecycle(
                                    topic_name=tp.name, node_id=ros_node.id, node_name=ros_node.name
                                ):
                                    new_transition_event_nodes.append(ros_node)
                            transition_event_publisher.append(ros_node.id)
                    elif is_request:
                        if ros_node.id not in tp.requester:
                            Log.debug(
                                f"{self.__class__.__name__}:      add requester {ros_node.id} "
                                f"{pub_info.node_namespace}/{pub_info.node_name}"
                            )
                            tp.requester.append(ros_node.id)
                    else:
                        # it is a publisher for a Reply service.
                        # We check whenever it is a composable service
                        if self._is_local_composable_service(tp.name, tp.srv_type, ros_node):
                            with self._lock:
                                ros_node.is_container = True
                                if ros_node.id not in self._composable_nodes:
                                    self._composable_nodes[ros_node.id] = RosComposable(ros_node.name, ros_node.id, [])
                                    new_composable_nodes.append(ros_node)
                                found_composable_nodes.append(ros_node.id)
                except Exception:
                    Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")

            sub_infos = nmd.ros_node.get_subscriptions_info_by_topic(topic_name, True)
            for sub_info in sub_infos:
                if "_NODE_NAME_UNKNOWN_" in sub_info.node_name or "_NODE_NAMESPACE_UNKNOWN_" in sub_info.node_namespace:
                    continue
                try:
                    if self._use_name_as_node_id:
                        gid = self._node_full_name(sub_info.node_namespace, sub_info.node_name)
                    else:
                        gid = self._guid_arr_to_str(sub_info.endpoint_gid[0:12])
                    t_gid = self._guid_arr_to_str(sub_info.endpoint_gid)
                    ros_node, is_new_node = self._get_node_from(
                        sub_info.node_namespace, sub_info.node_name, gid, cached_data
                    )
                except Exception:
                    Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")
                    continue
                if is_new_node:
                    result.append(ros_node)
                try:
                    tp = self._get_topic_from(topic_name, sub_info.topic_type, cached_data)
                    # topic or service ?
                    if isinstance(tp, RosTopic):
                        Log.debug(
                            f"{self.__class__.__name__}:      add subscriber {ros_node.id} "
                            f"{sub_info.node_namespace}/{sub_info.node_name} for {tp.name}"
                        )
                        sub_types.add(sub_info.topic_type)
                        topic_key = (tp.name, tp.msg_type)
                        subscriber_nodes = cached_data.topic_subscriber_nodes.setdefault(topic_key, set())
                        if ros_node.id not in subscriber_nodes:
                            # check QoS compatibility only against publishers with the same type
                            incompatible_qos = self._get_incompatible_qos(
                                pub_qos_by_type.get(sub_info.topic_type, []), sub_info.qos_profile, cached_data
                            )
                            endpoint_info = EndpointInfo(
                                t_gid, ros_node.id, self._get_qos(sub_info.qos_profile), incompatible_qos
                            )
                            tp.subscriber.append(endpoint_info)
                            subscriber_nodes.add(ros_node.id)
                        node_subscribers = cached_data.node_subscriber_ids.setdefault(ros_node.id, set())
                        if topic_key not in node_subscribers:
                            ros_node.subscribers.append(tp.get_topic_id())
                            node_subscribers.add(topic_key)
                    elif is_request and ros_node.id not in tp.provider:
                        Log.debug(
                            f"{self.__class__.__name__}:      add provider {ros_node.id} "
                            f"{sub_info.node_namespace}/{sub_info.node_name}"
                        )
                        tp.provider.append(ros_node.id)
                        # The node subscribes a Request service type. We add this service to the node.
                        # The node is now the provider of this service
                        service_key: TopicKey = (tp.name, tp.srv_type)
                        cached_data.service_provider_nodes.setdefault(service_key, set()).add(ros_node.id)
                        node_services = cached_data.node_service_ids.setdefault(ros_node.id, set())
                        if service_key not in node_services:
                            ros_node.services.append(tp.get_topic_id())
                            node_services.add(service_key)
                except Exception:
                    Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")

            # endpoints with different message types never connect; report this
            # as own warning instead of a (misleading) QoS incompatibility
            all_types = set(pub_qos_by_type) | sub_types
            if len(all_types) > 1:
                cached_data.type_mismatch[topic_name] = sorted(all_types)

        # get services if zenoh is enabled
        if self._use_name_as_node_id and hasattr(nmd.ros_node, "get_service_names_and_types_by_node"):
            self._add_services_by_node(result, cached_data, new_composable_nodes, found_composable_nodes)

        with self._lock:
            # cleanup the transition event subscriptions
            old_transitions_nodes = set(self._lifecycle_subscriptions.keys()) - set(transition_event_publisher)
            for node_id in old_transitions_nodes:
                sub = self._lifecycle_subscriptions.pop(node_id)
                Log.debug(f"{self.__class__.__name__}: unregister subscription {sub.topic_name}")
                try:
                    nmd.ros_node.destroy_subscription(sub)
                except Exception:
                    Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")

            # cleanup the composable nodes, containers with a pending request are kept
            old_container_nodes = (
                set(self._composable_nodes) - set(found_composable_nodes) - self._pending_composable_ids
            )
            for node_id in old_container_nodes:
                del self._composable_nodes[node_id]

        # update life cycle status using services, as messages may have been missed via topics
        for node in new_transition_event_nodes:
            self._queue_lifecycle_update(node.id, node.name)

        # check if existing containers need a refresh
        containers_to_refresh = self._has_new_unassigned_nodes(result)
        with self._lock:
            new_composable_ids = {n.id for n in new_composable_nodes}
            nodes_to_update: list[RosNode] = list(new_composable_nodes)
            for cid in containers_to_refresh:
                # avoid duplicates: only add if not already in new_composable_nodes
                if cid in self._composable_nodes and cid not in new_composable_ids:
                    nodes_to_update.append(RosNode(cid, self._composable_nodes[cid].containerName))
            self._pending_composable_ids.update(n.id for n in nodes_to_update)
            generation = self._state_generation
        if nodes_to_update:
            self._submit(self._thread_update_composables_call, nodes_to_update, 1, generation)

        # publish the collected state
        self._local_node_names = cached_data.local_node_names
        self._ros_service_dict = cached_data.service_objs
        self._ros_topic_dict = cached_data.topic_objs
        self._report_type_mismatch(cached_data)

        if self._shutdown:
            return
        if self._cb_nodes:
            self._cb_nodes(result)
        if self._cb_topics:
            self._cb_topics(cached_data.topic_objs)
        if self._cb_services:
            self._cb_services(cached_data.service_objs)

    def _add_services_by_node(
        self,
        result: list[RosNode],
        cached_data: CachedData,
        new_composable_nodes: list[RosNode],
        found_composable_nodes: list[NodeId],
    ) -> None:
        """Adds services reported per node. Required for rmw_zenoh_cpp, which does
        not expose the DDS service topics."""
        for node in result:
            node_base_name, node_ns = self.parse_node_name(node.name)
            try:
                service_names_and_types = nmd.ros_node.get_service_names_and_types_by_node(node_base_name, node_ns)
                for service_name, service_types in service_names_and_types:
                    for service_type in service_types:
                        srv = cached_data.service_objs.get((service_name, service_type))
                        if srv is None:
                            Log.debug(f"{self.__class__.__name__}:   create service {service_name} ({service_type})")
                            srv = RosService(service_name, service_type)
                            cached_data.service_objs[(service_name, service_type)] = srv
                        service_key: TopicKey = (srv.name, srv.srv_type)
                        provider_nodes = cached_data.service_provider_nodes.setdefault(service_key, set())
                        if node.id not in provider_nodes:
                            srv.provider.append(node.id)
                            provider_nodes.add(node.id)
                        node_services = cached_data.node_service_ids.setdefault(node.id, set())
                        if service_key not in node_services:
                            node.services.append(srv.get_topic_id())
                            node_services.add(service_key)
                        if self._is_local_composable_service(service_name, service_type, node):
                            with self._lock:
                                node.is_container = True
                                if node.id not in self._composable_nodes:
                                    self._composable_nodes[node.id] = RosComposable(node.name, node.id, [])
                                    new_composable_nodes.append(node)
                                found_composable_nodes.append(node.id)
            except Exception:
                Log.debug(
                    f"{self.__class__.__name__}: ignored exception for node "
                    f"{node_ns}/{node_base_name}: {traceback.format_exc()}"
                )

    def _report_type_mismatch(self, data: CachedData) -> None:
        """Reports topics used with more than one message type."""
        if not self.monitor_servicer:
            return
        if not data.type_mismatch:
            self.monitor_servicer.remove_warning_group(WARNING_GROUP_TOPIC_TYPES)
            return
        warnings_group = SystemWarningGroup(WARNING_GROUP_TOPIC_TYPES)
        for topic_name, types in data.type_mismatch.items():
            msg = (
                f"topic '{topic_name}' is used with different message types: {', '.join(types)}; "
                "these endpoints will never connect"
            )
            Log.warn(f"{self.__class__.__name__}: {msg}")
            warnings_group.append(SystemWarning(msg=msg))
        self.monitor_servicer.update_warning_groups([warnings_group])

    # ------------------------------------------------------------------ #
    #  graph object helper
    # ------------------------------------------------------------------ #

    def _is_request_topic(self, topic_name: str) -> IsRequest:
        return topic_name.startswith("rq/")

    def _append_local_node_name(self, data: CachedData, node_name: NodeFullName) -> None:
        """Adds a node name to the list of local nodes, duplicates are ignored."""
        if node_name not in data.local_node_name_set:
            data.local_node_name_set.add(node_name)
            data.local_node_names.append(node_name)

    def _get_incompatible_qos(
        self, pub_qos: list[QosPub], sub_qos_profile: Any, data: CachedData
    ) -> list[IncompatibleQos]:
        """Checks the QoS profile of a subscriber against all publishers of the same
        topic and type. Already checked QoS combinations are taken from the cache."""
        incompatible_qos: list[IncompatibleQos] = []
        if not pub_qos:
            return incompatible_qos
        sub_qos_key = self._qos_cache_key(sub_qos_profile)
        for qp in pub_qos:
            cache_key = None
            if sub_qos_key is not None and qp.qos_key is not None:
                cache_key = (qp.qos_key, sub_qos_key)
            if cache_key is not None and cache_key in data.qos_compatibility:
                compatibility, reason = data.qos_compatibility[cache_key]
            else:
                compatibility, reason = qos_check_compatible(qp.qos_profile, sub_qos_profile)
                if cache_key is not None:
                    data.qos_compatibility[cache_key] = (compatibility, reason)
            if compatibility != QoSCompatibility.OK:
                incompatible_qos.append(IncompatibleQos(qp.node.id, self._qos_compatibility2str(compatibility), reason))
        return incompatible_qos

    def _qos_cache_key(self, qos_profile: Any) -> QosKey:
        """Creates a hashable representation of a QoS profile.
        Returns None if the profile can not be converted, in this case no caching is applied."""
        try:
            return (
                int(qos_profile.durability),
                int(qos_profile.history),
                int(qos_profile.depth),
                int(qos_profile.liveliness),
                int(qos_profile.reliability),
                qos_profile.deadline.nanoseconds,
                qos_profile.liveliness_lease_duration.nanoseconds,
                qos_profile.lifespan.nanoseconds,
                bool(qos_profile.avoid_ros_namespace_conventions),
            )
        except Exception:
            return None

    def _resolve_participant_info(self, ros_node: RosNode) -> None:
        """Applies unicast locators/enclave of the DDS participant. Must be called
        before composable/lifecycle detection, which depends on 'is_local'."""
        participant = self._participant_infos.get(ros_node.gid)
        if participant is None:
            return
        ros_node.location = list(participant.unicast_locators)
        ros_node.enclave = participant.enclave
        if not ros_node.is_local and any(self.is_location_local(loc) for loc in ros_node.location):
            ros_node.is_local = True

    def _get_node_from(
        self, node_ns: NodeNamespace, node_name: NodeName, gid: ParticipantGid, data: CachedData
    ) -> tuple[RosNode, IsNew]:
        key = (node_ns, node_name, gid)
        ros_node = data.node_dict.get(key)
        if ros_node is not None:
            return ros_node, False

        full_name = self._node_full_name(node_ns, node_name)
        Log.debug(f"{self.__class__.__name__}:   create node: {full_name}")
        node_id = full_name if full_name == gid else f"{full_name}-{gid}"
        ros_node = RosNode(node_id, full_name)
        ros_node.namespace = node_ns
        ros_node.gid = gid
        # add active screens of this node
        process_ids: list[int] = []
        for session_name in data.screens_by_node.get(full_name, ()):
            Log.debug(f"{self.__class__.__name__}:     append screen: {session_name}")
            ros_node.screens.append(session_name)
            try:
                process_ids.append(int(session_name.split(".")[0]))
            except ValueError:
                Log.debug(f"{self.__class__.__name__}: can not parse pid from screen session {session_name}")
        # try to find the process of the node, uses the process list cached for this update cycle
        process_ids.extend(self.find_node(full_name, data.processes))
        # remove duplicates, keep the order
        ros_node.process_ids = list(dict.fromkeys(process_ids))
        ros_node.system_node = names.basename(full_name).startswith("_") or full_name in ["/rosout"]
        ros_node.system_node |= node_ns == "/mas" or node_ns.startswith("/mas/")
        # if a process/screen is available, we assume it is a local node
        if ros_node.process_ids:
            ros_node.location = list(self._local_addresses)
            ros_node.is_local = True
        elif full_name in data.composable_node_names:
            # nodes inside a local container are local too
            ros_node.is_local = True
        else:
            with self._lock:
                self._resolve_participant_info(ros_node)
        if ros_node.is_local:
            self._append_local_node_name(data, full_name)
        data.node_dict[key] = ros_node
        return ros_node, True

    def _get_topic_from(self, topic_name: str, topic_type: str, data: CachedData) -> RosTopic | RosService:
        if topic_name.startswith(DDS_TOPIC_PREFIX):
            name = topic_name[2:]
            key = (name, topic_type)
            result_obj = data.topic_objs.get(key)
            if result_obj is None:
                topic_type_res = self.get_message_type(topic_type)
                Log.debug(f"{self.__class__.__name__}:   create topic {name} ({topic_type_res})")
                result_obj = RosTopic(name, topic_type_res)
                data.topic_objs[key] = result_obj
            return result_obj
        if topic_name.startswith(DDS_SERVICE_PREFIXES):
            srv_type = self.get_service_type(topic_type)
            # TODO: distinction between Reply/Request? Currently it is removed.
            srv_name = self.get_service_name(topic_name[2:])
            key = (srv_name, srv_type)
            result_srv = data.service_objs.get(key)
            if result_srv is None:
                Log.debug(f"{self.__class__.__name__}:   create service {srv_name} ({srv_type})")
                result_srv = RosService(srv_name, srv_type)
                data.service_objs[key] = result_srv
            return result_srv
        # fallback if we have no DDS prefix, e.g. while using zenoh
        key = (topic_name, topic_type)
        result_obj = data.topic_objs.get(key)
        if result_obj is None:
            topic_type_res = self.get_message_type(topic_type)
            Log.debug(f"{self.__class__.__name__}:   create topic {topic_name} ({topic_type_res})")
            result_obj = RosTopic(topic_name, topic_type_res)
            data.topic_objs[key] = result_obj
        return result_obj

    def _is_local_composable_service(
        self, service_name: ServiceNameWoPrefix, service_type: ServiceType, ros_node: RosNode
    ) -> bool:
        # cheap checks first
        if not service_name.endswith("/_container/list_nodes"):
            return False
        if service_type != "composition_interfaces/srv/ListNodes":
            return False
        return ros_node.is_local

    def _is_local_lifecycle_transition_topic(
        self, topic_name: TopicNameWoPrefix, topic_type: TopicType, ros_node: RosNode, data: CachedData
    ) -> bool:
        """Checks whether the topic is a lifecycle event topic of a local node.
        Nodes running inside a local composable container count as local."""
        try:
            # cheap checks first
            if not topic_name.endswith("/transition_event"):
                return False
            if topic_type != "lifecycle_msgs/msg/TransitionEvent":
                return False
            if ros_node.is_local:
                return True
            return ros_node.name in data.composable_node_names
        except Exception:
            Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")
        return False

    def _get_qos(self, qos: QoSProfile) -> RosQos:
        return RosQos(
            qos.durability,
            qos.history,
            qos.depth,
            qos.liveliness,
            qos.reliability,
            qos.deadline,
            qos.liveliness_lease_duration,
            qos.lifespan,
            qos.avoid_ros_namespace_conventions,
        )

    def _qos_compatibility2str(self, qc: QoSCompatibility) -> str:
        if qc == QoSCompatibility.OK:
            return "ok"
        return "warning"

    def _publish_warnings(self, error_msgs: list[str]) -> None:
        if not error_msgs:
            return
        warnings_group = SystemWarningGroup(SystemWarningGroup.ID_ROS_STATE)
        for msg in error_msgs:
            warnings_group.append(SystemWarning(msg=msg))
            Log.warn(msg)
        if self.monitor_servicer:
            self.monitor_servicer.update_warning_groups([warnings_group])

    # ------------------------------------------------------------------ #
    #  asynchronous service calls
    # ------------------------------------------------------------------ #

    def _thread_update_lifecycle_call(self, nodes: list[RosNode]):
        """Updates the lifecycle state of the specified nodes by calling the
        services /get_state and /get_available_transitions."""
        if not nodes or not LIFECYCLE_AVAILABLE or self._shutdown:
            return
        error_msgs: list[str] = []
        try:
            wait_futures: list[WaitFuture] = []
            for node in nodes:
                Log.debug(f"{self.__class__.__name__}:  update lifecycle state '{node.name}'")
                create_service_future(
                    nmd.ros_node,
                    wait_futures=wait_futures,
                    type="lifecycle state",
                    node_id=node.id,
                    node_name=node.name,
                    service_name=f"{node.name}/get_state",
                    srv_type=GetState,
                    request=GetState.Request(),
                    callback_group=self._callback_group,
                )
                Log.debug(f"{self.__class__.__name__}:  update lifecycle transitions '{node.name}'")
                create_service_future(
                    nmd.ros_node,
                    wait_futures=wait_futures,
                    type="lifecycle transition",
                    node_id=node.id,
                    node_name=node.name,
                    service_name=f"{node.name}/get_available_transitions",
                    srv_type=GetAvailableTransitions,
                    request=GetAvailableTransitions.Request(),
                    callback_group=self._callback_group,
                )
            if not wait_futures:
                return
            # wait until all services are finished or timed out
            wait_until_futures_done(wait_futures, SERVICE_CALL_TIMEOUT)
            # handle response
            lifecycle_states: dict[NodeId, RosLifecycleState] = {}
            for wait_future in wait_futures:
                if wait_future.node_id not in lifecycle_states:
                    lifecycle_states[wait_future.node_id] = RosLifecycleState(
                        id=wait_future.node_id, name=wait_future.node_name
                    )
            for wait_future in wait_futures:
                lifecycle_state = lifecycle_states[wait_future.node_id]
                if wait_future.finished:
                    try:
                        response = wait_future.future.result()
                        if response:
                            if wait_future.type == "lifecycle state":
                                lifecycle_state.state = response.current_state.label
                            elif wait_future.type == "lifecycle transition":
                                for transition in response.available_transitions:
                                    lifecycle_state.available_transitions.append(
                                        LifecycleTransition(transition.transition.label, transition.transition.id)
                                    )
                    except Exception as exception:
                        error_msgs.append(
                            f"{self.__class__.__name__}:-> failed to update {wait_future.type} "
                            f"of '{wait_future.node_name}': '{exception}'"
                        )
                else:
                    error_msgs.append(
                        f"{self.__class__.__name__}:-> Timeout while update {wait_future.type} "
                        f"of '{wait_future.node_name}'"
                    )
                wait_future.client.destroy()
            # callback, called outside of the lock to avoid dead locks
            if not self._shutdown and self._cb_lifecycle:
                self._cb_lifecycle(list(lifecycle_states.values()))
        except Exception:
            error_msgs.append(traceback.format_exc())
        finally:
            self._publish_warnings(error_msgs)

    def _thread_update_composables_call(self, nodes: list[RosNode], retry: int = 1, generation: int = -1):
        """Updates the list of composable nodes in the specified container nodes."""
        error_msgs: list[str] = []
        node_ids = {node.id for node in nodes}
        try:
            if self._shutdown:
                return
            wait_futures: list[WaitFuture] = []
            for node in nodes:
                Log.debug(f"{self.__class__.__name__}:  update composables nodes for '{node.name}'")
                create_service_future(
                    nmd.ros_node,
                    wait_futures=wait_futures,
                    type="composable",
                    node_id=node.id,
                    node_name=node.name,
                    service_name=f"{node.name}/_container/list_nodes",
                    srv_type=ListNodes,
                    request=ListNodes.Request(),
                    callback_group=self._callback_group,
                )
            if not wait_futures:
                return
            # wait until all services are finished or timed out
            wait_until_futures_done(wait_futures, SERVICE_CALL_TIMEOUT)
            # handle response
            retry_nodes: list[RosNode] = []
            for wait_future in wait_futures:
                if wait_future.finished and wait_future.type == "composable":
                    try:
                        response = wait_future.future.result()
                        if response is not None:
                            composable = RosComposable(
                                container_name=wait_future.node_name, node_id=wait_future.node_id
                            )
                            composable.nodes.extend(response.full_node_names)
                            composable.composableIds = list(zip(response.full_node_names, response.unique_ids))
                            with self._lock:
                                # discard results of a state which was reset in the meantime
                                if generation < 0 or generation == self._state_generation:
                                    self._composable_nodes[wait_future.node_id] = composable
                    except Exception as exception:
                        error_msgs.append(
                            f"{self.__class__.__name__}:-> failed to update composable nodes "
                            f"of '{wait_future.node_name}': '{exception}'"
                        )
                elif not wait_future.finished:
                    if retry < COMPOSABLE_RETRY_MAX:
                        # retry for the node of this future, not for the last node of the loop
                        retry_nodes.append(RosNode(wait_future.node_id, wait_future.node_name))
                    else:
                        error_msgs.append(
                            f"{self.__class__.__name__}:-> Timeout while update "
                            f"{wait_future.type} of '{wait_future.node_name}'"
                        )
                wait_future.client.destroy()

            if retry_nodes:
                Log.debug(f"{self.__class__.__name__}: RETRY {retry} for {len(retry_nodes)} nodes")
                retry_ids = {node.id for node in retry_nodes}
                with self._lock:
                    # keep the retried containers marked as pending
                    self._pending_composable_ids.update(retry_ids)
                node_ids -= retry_ids
                self._submit(self._thread_update_composables_call, retry_nodes, retry + 1, generation)

            # create a snapshot and call the callback outside of the lock to avoid dead locks
            with self._lock:
                composables = list(self._composable_nodes.values())
            if not self._shutdown and self._cb_composables:
                self._cb_composables(composables)
        except Exception:
            error_msgs.append(traceback.format_exc())
        finally:
            for wait_future in wait_futures:
                try:
                    wait_future.client.destroy()
                except Exception:
                    Log.debug(f"{self.__class__.__name__}: ignored exception: {traceback.format_exc()}")
            with self._lock:
                self._pending_composable_ids -= node_ids
            self._publish_warnings(error_msgs)
