# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from .launch.file_observer import FileObserverRegistry, default_watch_roots
from .launch.launch_argument_cache import LAUNCH_ARGUMENT_CACHE
from .launch.launch_config import LaunchConfig
from .launch.launch_context import LaunchContext
from .launch.launch_validator import LaunchValidator
from functools import lru_cache
from .launch.caches import FILE_CONTENT_CACHE
from .launch.caches import MESSAGE_STRUCT_CACHE
from .launch.caches import normalize_path
from .launch.caches import cache_statistics
import fkie_mas_daemon as nmd
import csv
import json
import os
import re
import shlex
import sys
import traceback
from threading import Lock
from threading import RLock
from importlib import import_module

from typing import Dict
from typing import FrozenSet
from typing import List
from typing import Optional
from typing import Tuple
from typing import Type

from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.utilities import get_service
from rosidl_runtime_py.utilities import get_action
from rosidl_runtime_py import get_service_interfaces
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import get_action_interfaces
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py import message_to_ordereddict
import rosidl_parser.definition

from fkie_mas_pylib.websocket.server import WebSocketServer
from fkie_mas_pylib.system.supervised_popen import SupervisedPopen
from fkie_mas_pylib.system.url import equal_uri
from fkie_mas_pylib.system.host import is_local
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.system import exceptions
from fkie_mas_pylib.names import ns_join
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.launch import xml
from fkie_mas_pylib.defines import SEARCH_IN_EXT
from fkie_mas_pylib.defines import ros2_publisher_nodename_tuple
from fkie_mas_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_mas_pylib.defines import ros2_action_nodename_tuple
from fkie_mas_pylib.defines import ros2_action_introspection_nodename_tuple
from fkie_mas_pylib.defines import ros2_service_introspection_nodename_tuple
from fkie_mas_pylib.interface.launch_interface import LaunchPublishMessage
from fkie_mas_pylib.interface.launch_interface import LaunchMessageStruct
from fkie_mas_pylib.interface.launch_interface import LaunchIncludedFile
from fkie_mas_pylib.interface.launch_interface import LaunchIncludedFilesRequest
from fkie_mas_pylib.interface.launch_interface import LaunchInterpretPathReply
from fkie_mas_pylib.interface.launch_interface import LaunchInterpretPathRequest
from fkie_mas_pylib.interface.launch_interface import LaunchNodeReply
from fkie_mas_pylib.interface.launch_interface import LaunchNode
from fkie_mas_pylib.interface.launch_interface import LaunchAssociations
from fkie_mas_pylib.interface.launch_interface import LaunchContent
from fkie_mas_pylib.interface.launch_interface import LaunchLoadReply
from fkie_mas_pylib.interface.launch_interface import LaunchLoadRequest
from fkie_mas_pylib.interface.launch_interface import LaunchFile
from fkie_mas_pylib.interface.launch_interface import LaunchCallService
from fkie_mas_pylib.interface.launch_interface import LaunchArgument
from fkie_mas_pylib.interface.launch_interface import RosRun
from fkie_mas_pylib.interface.launch_interface import RosRunReply
from fkie_mas_pylib.interface.runtime_interface import RosQos
from fkie_mas_pylib.interface.runtime_interface import SubscriberNode
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib import ros_pkg

from .launch.launch_definition_index import invalidate_node_definitions
from .launch.launch_include_index import invalidate_include_definitions
ActionClass = Type
ActionRequestClass = Type

# precompiled patterns: used per node on every reload / per field on every struct
PARAMS_FILE_RE = re.compile(r'--params-file\s+([^\s]+)')
ANONYMOUS_NODE_RE = re.compile(r"\d{3,6}_\d{10,}")
SEQUENCE_TYPE_RE = re.compile(r'<(\w[^,]*),?\s*(\S*)>')
ARRAY_TYPE_RE = re.compile(r'(.*)\[(\d*)\]')

# simple type conversion dispatch, checked in this order
_TYPE_CONVERTERS = (
    ('int', int),
    ('octet', int),
    ('float', float),
    ('double', float),
)


class CfgId(object):
    '''
    Identification object for a loaded launch file. You can load the same launch file for different ROS-Master!
    '''

    def __init__(self, path: str, daemonuri: str = ''):
        '''
        :param str path: absolute path of the launch file.
        :param str daemonuri: daemon where to launch the configuration
        '''
        self.path = path
        self.daemonuri = daemonuri
        self._local = is_local(daemonuri)

    def __str__(self):
        return "%s%s" % (self.daemonuri, self.path)

    def __repr__(self):
        return "%s%s" % (self.daemonuri, self.path)

    def __hash__(self, *args, **kwargs):
        return hash("%s%s" % (self.daemonuri, self.path))

    def __eq__(self, other):
        '''
        Compares the path of the item.
        '''
        if isinstance(other, tuple):
            return self.path == other[0] and self.equal_hosts(other[1])
        elif other is not None:
            return self.path == other.path and self.equal_hosts(other.daemonuri)
        return False

    def __ne__(self, other):
        return not (self == other)

    def equal_hosts(self, daemonuri: str):
        '''
        Compares the daemonuri names of this instance with other host.

        :param str daemonuri: uri of other daemon
        '''
        if not daemonuri:
            if self._local:
                return True
        if equal_uri(self.daemonuri, daemonuri):
            return True
        return False


class LaunchServicer:
    '''
    Websocket service methods around loaded ROS2 launch files.
    '''

    # Lock hierarchy, never acquire in reverse order:
    #   1. self._loaded_files_lock
    #   2. FileObserverRegistry internal lock
    #   3. LaunchConfig._LOAD_LOCK  (os.environ, launch file parsing)
    # self._peers_lock, self._node_exec_lock and self._observer_launch_lock are
    # leaf locks: they are never held while any other lock is acquired and in
    # particular never while calling into the observer.

    def __init__(self, websocket: WebSocketServer, ws_port: int):
        Log.info("Create ROS2 launch servicer")
        self._loaded_files_lock = RLock()
        self._node_exec_lock = Lock()
        self._peers_lock = Lock()
        self._loaded_files: Dict[CfgId, LaunchConfig] = {}
        self._node_exec: Dict[str, str] = {}   # node name -> executable path
        self._peers: Dict[str, object] = {}
        self._is_running = True
        self.ws_port = ws_port
        self.websocket = websocket
        self.xml_validator = LaunchValidator()
        self._callback_service_group = ReentrantCallbackGroup()
        roots = default_watch_roots()
        self._observer = FileObserverRegistry(self._on_file_changed, watch_roots=roots)
        Log.info(f"file observer watch roots: {roots}")
        # Observer registration IDs are unique for a launch path and daemon URI.
        self._observer_launch_lock = Lock()
        self._observer_launch_paths: Dict[str, str] = {}
        self._observer.start()

        websocket.register("ros.launch.load", self.load_launch)
        websocket.register("ros.launch.reload", self.reload_launch)
        websocket.register("ros.launch.unload", self.unload_launch)
        websocket.register("ros.launch.get_list", self.get_list)
        websocket.register("ros.launch.ros_run", self.ros_run)
        websocket.register("ros.launch.start_node", self.start_node)
        websocket.register("ros.launch.start_nodes", self.start_nodes)
        websocket.register("ros.launch.get_included_files", self.get_included_files)
        websocket.register("ros.launch.interpret_path", self.interpret_path)
        websocket.register("ros.launch.get_msg_struct", self.get_msg_struct)
        websocket.register("ros.launch.publish_message", self.publish_message)
        websocket.register("ros.launch.get_srv_struct", self.get_srv_struct)
        websocket.register("ros.launch.call_service", self.call_service)
        websocket.register("ros.launch.get_message_types", self.get_message_types)
        websocket.register("ros.publisher.start", self.publish_message)
        websocket.register("ros.subscriber.start", self.start_subscriber)
        websocket.register("ros.action.send_goal", self.start_action)
        websocket.register("ros.action.introspection.start", self.start_action_introspection)
        websocket.register("ros.service.introspection.start", self.start_service_introspection)
        websocket.register("ros.daemon.get_cache_statistics", self.get_cache_statistics)

    def stop(self):
        '''Stop the file observer.'''
        self._is_running = False
        with self._observer_launch_lock:
            self._observer_launch_paths.clear()
        self._observer.stop()

    def _terminated(self):
        Log.info(f"{self.__class__.__name__}: terminated launch context")

    @staticmethod
    def _observer_launch_id(launch_config: LaunchConfig) -> str:
        """
        Return a unique observer registration ID.

        The same launch file may be loaded for different daemon URIs. The observer
        therefore needs a unique registration key for every loaded configuration.
        """
        return (
            f"{launch_config.daemonuri}\x1f"
            f"{normalize_path(launch_config.filename)}"
        )

    def get_cache_statistics(self) -> str:
        '''Hit rates and sizes of all launch related caches.'''
        return json.dumps(cache_statistics(), cls=SelfEncoder)

    def _register_callback(self, context):
        peer = context.peer()
        with self._peers_lock:
            if peer in self._peers:
                return
        Log.info(f"{self.__class__.__name__}: Add callback to peer context @{peer}")
        if context.add_callback(self._terminated):
            with self._peers_lock:
                self._peers[peer] = context

    # -- access to _loaded_files -------------------------------------------

    def _get_config(self, path: str, daemonuri: str = '') -> Optional[LaunchConfig]:
        with self._loaded_files_lock:
            return self._loaded_files.get(CfgId(path, daemonuri))

    def _set_config(self, cfgid: CfgId, launch_config: LaunchConfig) -> None:
        with self._loaded_files_lock:
            self._loaded_files[cfgid] = launch_config

    def _pop_config(self, cfgid: CfgId) -> Optional[LaunchConfig]:
        with self._loaded_files_lock:
            return self._loaded_files.pop(cfgid, None)

    def _snapshot_configs(self) -> List[Tuple[CfgId, LaunchConfig]]:
        '''Stable snapshot, so iteration never holds the lock.'''
        with self._loaded_files_lock:
            return list(self._loaded_files.items())

    # -- observer ----------------------------------------------------------

    def _observe_launch(self, launch_config: LaunchConfig) -> List[str]:
        '''Observe a launch file and all its included files.'''
        paths = [launch_config.filename]

        try:
            request = LaunchIncludedFilesRequest(launch_config.filename,
                                                 args=launch_config.launch_arguments
                                                 )

            # Pass the daemon URI, otherwise the resolved include list of a
            # configuration loaded for a remote daemon would not be found.
            for inc_description in self.get_included_files(
                    request, result_as_json=False,
                    daemonuri=launch_config.daemonuri):
                if inc_description.inc_path:
                    paths.append(inc_description.inc_path)

        except Exception as error:
            Log.error(
                f"{self.__class__.__name__}: cannot determine included files "
                f"of {launch_config.filename}: {error}"
            )
            return [f"{launch_config.filename}: {error}"]

        # Remove duplicates while preserving the original order.
        normalized_paths = list(
            dict.fromkeys(
                normalize_path(path)
                for path in paths
                if path
            )
        )

        observer_id = self._observer_launch_id(launch_config)

        with self._observer_launch_lock:
            self._observer_launch_paths[observer_id] = normalize_path(launch_config.filename)

        try:
            # register_launch() replaces the whole path set of this ID atomically,
            # so it can also be used to update an already registered launch file.
            return self._observer.register_launch(observer_id, normalized_paths)
        except Exception:
            with self._observer_launch_lock:
                self._observer_launch_paths.pop(observer_id, None)
            raise

    def _unobserve_launch(self, launch_config: LaunchConfig) -> None:
        """
        Remove exactly the observer registration belonging to this configuration.
        """
        observer_id = self._observer_launch_id(launch_config)

        try:
            self._observer.unregister_launch(observer_id)
        finally:
            with self._observer_launch_lock:
                self._observer_launch_paths.pop(observer_id, None)

    def _on_file_changed(self, event_type: str, path: str,
                         affected_launch_files: FrozenSet[str]) -> None:
        '''
        Handle a file-system event.

        The observer returns internal registration IDs. They are translated back
        to launch file paths before cache invalidation and websocket publishing.
        '''
        if not self._is_running:
            return

        with self._observer_launch_lock:
            affected_paths = frozenset(
                self._observer_launch_paths[observer_id]
                for observer_id in affected_launch_files
                if observer_id in self._observer_launch_paths
            )

        # Invalidate content derived directly from the changed file.
        FILE_CONTENT_CACHE.invalidate(path)
        LAUNCH_ARGUMENT_CACHE.invalidate(path)
        invalidate_node_definitions(path)
        invalidate_include_definitions(path)

        # Invalidate argument templates of affected root launch files.
        for launch_path in affected_paths:
            LAUNCH_ARGUMENT_CACHE.invalidate(launch_path)

        Log.debug(
            f"{self.__class__.__name__}: observed change "
            f"{event_type} on {path}, affected: "
            f"{sorted(affected_paths)}"
        )

        self.websocket.publish('ros.path.changed',
                               {
                                   "eventType": event_type,
                                   "srcPath": path,
                                   "affected": sorted(affected_paths),
                               }
                               )

    # -- environment -------------------------------------------------------

    @staticmethod
    def _env_snapshot() -> Dict[str, str]:
        '''Consistent copy of os.environ, serialized against launch parsing.'''
        with LaunchConfig._LOAD_LOCK:
            return dict(os.environ)

    @classmethod
    def _local_env(cls) -> Dict[str, str]:
        '''Environment for locally started nodes with normalized DISPLAY.'''
        new_env = cls._env_snapshot()
        if 'DISPLAY' in new_env:
            if not new_env['DISPLAY'] or new_env['DISPLAY'] == 'remote':
                del new_env['DISPLAY']
        else:
            new_env['DISPLAY'] = ':0'
        return new_env

    def load_launch(self, request_json: LaunchLoadRequest, *, requester: str = "",
                    return_as_json: bool = True) -> LaunchLoadReply:
        '''Loads launch file by interface request'''
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.load]")
        request = request_json
        result = LaunchLoadReply()
        # request scoped state, never stored on self
        launchfile = request.path
        daemonuri = getattr(request, 'masteruri', '') or ''
        Log.debug(f"{self.__class__.__name__}: Loading launch file: {launchfile} "
                  f"(package: {request.ros_package}, launch: {request.launch}), "
                  f"daemonuri: {daemonuri}, host: {request.host}, args: {request.args}")

        if not launchfile:
            resolved = self._resolve_package_launch(request, result)
            if resolved is None:
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
            launchfile = resolved
        result.paths.append(launchfile)

        cfgid = CfgId(launchfile, daemonuri)
        if self._get_config(launchfile, daemonuri) is not None:
            result.status.code = 'ALREADY_OPEN'
            result.status.msg = f"Launch file {launchfile} already loaded!"
            Log.debug(f"{self.__class__.__name__}: ..load aborted, ALREADY_OPEN")
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result

        try:
            provided_arg_names = [arg.name for arg in request.args]
            launch_context = LaunchContext(argv=sys.argv[1:])
            provided_args = [] if request.args is None else request.args
            req_args = list(provided_args)
            if not req_args:
                req_args = LaunchConfig.get_launch_arguments(
                    launch_context, launchfile,
                    provided_args=None if request.request_args else provided_args)
                if request.request_args and req_args:
                    missing = [arg for arg in req_args if arg.name not in provided_arg_names]
                    if missing:
                        result.args.extend(req_args)
                        result.status.code = 'PARAMS_REQUIRED'
                        Log.debug(f"{self.__class__.__name__}: ..load aborted, PARAMS_REQUIRED "
                                  f"{[arg.name for arg in result.args]}; "
                                  f"provided args {provided_arg_names}")
                        return json.dumps(result, cls=SelfEncoder) if return_as_json else result
            launch_arguments = [(arg.name, arg.value) if hasattr(arg, "value")
                                else (arg.name, arg.default_value) for arg in req_args]
            # heavy work: no servicer lock is held, LaunchConfig serializes the
            # os.environ access on its own class lock
            launch_config = LaunchConfig(launchfile, context=launch_context,
                                         daemonuri=daemonuri,
                                         launch_arguments=launch_arguments)
        except Exception as error:
            print(traceback.format_exc())
            err_details = f"{launchfile} loading failed!: {error}"
            Log.warn(f"{self.__class__.__name__}: Loading launch file: {err_details}")
            result.status.code = 'ERROR'
            result.status.msg = err_details
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result

        # insert only if no concurrent request won the race meanwhile
        with self._loaded_files_lock:
            if cfgid in self._loaded_files:
                result.status.code = 'ALREADY_OPEN'
                result.status.msg = f"Launch file {launchfile} already loaded!"
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
            self._loaded_files[cfgid] = launch_config

        # observer and publish strictly after the lock is released
        try:
            observer_warnings = sorted(set(self._observe_launch(launch_config)))
        except Exception as error:
            # A failing observer must not abort an otherwise successful load.
            observer_warnings = [f"{launchfile}: cannot observe files: {error}"]
            Log.warn(f"{self.__class__.__name__}: observing {launchfile} failed:\n"
                     f"{traceback.format_exc()}")

        messages = list(observer_warnings)
        if launch_config.load_exceptions:
            messages.insert(0, launch_config.load_exceptions[0])
        result.status.msg = '\n'.join(messages)
        launch_config.load_exceptions.extend(observer_warnings)
        result.env = self._env_snapshot()
        result.status.code = 'OK'
        Log.debug(f"{self.__class__.__name__}: ..load complete!")
        try:
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        finally:
            self.websocket.publish('ros.launch.changed',
                                   {'path': launchfile, 'action': 'loaded',
                                    'requester': requester})

    def _resolve_package_launch(self, request: LaunchLoadRequest,
                                result: LaunchLoadReply) -> Optional[str]:
        '''Determine the launch file path from package and launch name.
        Returns None and fills result.status on error.'''
        try:
            paths = ros_pkg.get_share_files_path_from_package(
                request.ros_package, request.launch)
        except LookupError as rnf:
            result.status.code = 'FILE_NOT_FOUND'
            result.status.msg = f"Package {request.ros_package} not found: {rnf}"
            Log.debug(f"{self.__class__.__name__}: ..load aborted, FILE_NOT_FOUND")
            return None
        if not paths:
            result.status.code = 'FILE_NOT_FOUND'
            result.status.msg = (f"Launch files {request.launch} in package "
                                 f"{request.ros_package} not found!")
            return None
        if len(paths) > 1 and not request.force_first_file:
            result.status.code = 'MULTIPLE_LAUNCHES'
            result.status.msg = (f"Multiple launch files with name {request.launch} "
                                 f"in package {request.ros_package} found!")
            result.paths.extend(paths)
            Log.debug(f"{self.__class__.__name__}: ..load aborted, MULTIPLE_LAUNCHES")
            return None
        return paths[0]

    def reload_launch(self, request_json: LaunchLoadRequest, *,
                      requester: str = "") -> LaunchLoadReply:
        '''Reloads launch file by interface request'''
        Log.info(f"{self.__class__.__name__}: Request to [ros.launch.reload]")
        request = request_json
        result = LaunchLoadReply()
        daemonuri = getattr(request, 'masteruri', '') or ''
        result.paths.append(request.path)
        cfgid = CfgId(request.path, daemonuri)
        Log.debug(f"{self.__class__.__name__}: reload launch file: {request.path}, "
                  f"daemonuri: {daemonuri}")

        old_launch = self._get_config(request.path, daemonuri)
        if old_launch is None:
            result.status.code = 'FILE_NOT_FOUND'
            return json.dumps(result, cls=SelfEncoder)

        try:
            # The old registration is intentionally kept while parsing: the
            # observer ID of the old and the new configuration is identical, so
            # _observe_launch() below replaces the observed path set atomically
            # and no change is missed during the reload.
            launch_context = LaunchContext(argv=sys.argv[1:])
            # keep the values of the currently provided arguments, new arguments
            # of the changed file get their default value
            provided = {name: value for name, value in old_launch.provided_launch_arguments}
            req_args: List[LaunchArgument] = LaunchConfig.get_launch_arguments(
                launch_context, old_launch.filename, provided_args=[])
            launch_arguments = [(arg.name, provided.get(arg.name, arg.value))
                                for arg in req_args]
            launch_config = LaunchConfig(old_launch.filename, context=launch_context,
                                         daemonuri=daemonuri,
                                         launch_arguments=launch_arguments)
            self._set_config(cfgid, launch_config)
            result.status.code = 'OK'
            result.changed_nodes.extend(
                self._changed_nodes(old_launch.nodes(), launch_config.nodes()))
            old_launch.unload()
            self._observe_launch(launch_config)
            if launch_config.load_exceptions:
                result.status.msg = launch_config.load_exceptions[0]
        except Exception as error:
            # restore the previous configuration
            self._set_config(cfgid, old_launch)
            old_launch.load()
            try:
                self._observe_launch(old_launch)
            except Exception:
                Log.warn(f"{self.__class__.__name__}: cannot restore observation of "
                         f"{old_launch.filename}:\n{traceback.format_exc()}")
            print(traceback.format_exc())
            err_details = f"{request.path} loading failed!: {error}"
            Log.warn(f"{self.__class__.__name__}: Loading launch file: {err_details}")
            result.status.code = 'ERROR'
            result.status.msg = err_details
            return json.dumps(result, cls=SelfEncoder)
        try:
            return json.dumps(result, cls=SelfEncoder)
        finally:
            self.websocket.publish('ros.launch.changed',
                                   {'path': request.path, 'action': 'reloaded',
                                    'requester': requester})

    @classmethod
    def _changed_nodes(cls, old_nodes: List, new_nodes: List) -> List[str]:
        '''Node names that have to be restarted after a reload.
        Anonymous nodes are filtered out, their name changes on every load.'''
        old_by_name = {node.node_name: node for node in old_nodes}
        changed = []
        for new_node in new_nodes:
            old_node = old_by_name.get(new_node.node_name)
            if old_node is None or cls._node_needs_restart(new_node, old_node):
                changed.append(new_node.node_name)
        return [name for name in changed if not ANONYMOUS_NODE_RE.search(name)]

    @staticmethod
    def _node_needs_restart(new_node, old_node) -> bool:
        if new_node.additional_env and old_node.additional_env:
            if set(new_node.additional_env.values()) - set(old_node.additional_env.values()):
                return True
            if set(new_node.additional_env) - set(old_node.additional_env):
                return True
        if new_node.launch_prefix != old_node.launch_prefix:
            return True
        if new_node.cmd == old_node.cmd:
            return False
        new_matches = PARAMS_FILE_RE.findall(new_node.cmd)
        old_matches = PARAMS_FILE_RE.findall(old_node.cmd)
        if len(new_matches) != len(old_matches):
            return True
        # the command differs: compare the content of the parameter files, the
        # file names contain a random part on every launch
        new_content = getattr(new_node, 'param_file_content', {}) or {}
        old_content = getattr(old_node, 'param_file_content', {}) or {}
        normalized_new = new_node.cmd
        normalized_old = old_node.cmd
        for new_file, old_file in zip(new_matches, old_matches):
            if new_content.get(new_file, "") != old_content.get(old_file, ""):
                return True
            normalized_new = normalized_new.replace(new_file, '')
            normalized_old = normalized_old.replace(old_file, '')
        return normalized_new != normalized_old

    def unload_launch(self, request_json: LaunchFile, *, requester: str = "") -> LaunchLoadReply:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.unload]")
        request = request_json
        Log.debug(f"{self.__class__.__name__}: Unload Launch request:\n {request}")
        result = LaunchLoadReply()
        result.paths.append(request.path)
        # TODO: check if we need daemonuri as identification
        daemonuri = getattr(request, 'masteruri', '') or ''
        cfgid = CfgId(request.path, daemonuri)

        try:
            launch_config = self._pop_config(cfgid)
            if launch_config is None:
                result.status.code = 'FILE_NOT_FOUND'
                result.status.msg = f"{request.path} not found"
            else:
                # observer calls outside of the lock
                self._unobserve_launch(launch_config)
                result.status.code = 'OK'
        except Exception as error:
            err_details = f"{request.path} unloading failed!: {error}"
            Log.warn(f"{self.__class__.__name__}: Unloading launch file: {err_details}")
            result.status.code = 'ERROR'
            result.status.msg = err_details

        try:
            return json.dumps(result, cls=SelfEncoder)
        finally:
            self.websocket.publish('ros.launch.changed',
                                   {'path': request.path,
                                    'action': 'unloaded',
                                    'requester': requester}
                                   )

    def get_list(self) -> List[LaunchContent]:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.get_list]")
        reply = []
        for cfgid, lc in self._snapshot_configs():
            reply_lc = LaunchContent(path=cfgid.path, args=[], masteruri=lc.daemonuri,
                                     host='', nodes=[], parameters=[], associations=[])
            for name, p in lc.provided_launch_arguments:
                reply_lc.args.append(
                    LaunchArgument(name, p.value if hasattr(p, 'value') else p))
            for item in lc.nodes():
                reply_lc.nodes.append(item)
                for association in self._node_associations(item):
                    reply_lc.associations.append(association)
            reply_lc.warnings = lc.load_exceptions
            reply_lc.env = lc.environment
            reply.append(reply_lc)
        return json.dumps(reply, cls=SelfEncoder)

    @staticmethod
    def _node_associations(item) -> List[LaunchAssociations]:
        result = []
        for p in item.parameters:
            associations = None
            if p.name == "mas/associations":
                associations = p.value
            elif isinstance(p.value, dict):
                associations = (p.value.get("/**", {})
                                .get("ros__parameters", {})
                                .get("mas/associations"))
                if associations is None:
                    associations = (p.value.get(item.node_name, {})
                                    .get("ros__parameters", {})
                                    .get("mas/associations"))
            if associations is not None:
                result.append(LaunchAssociations(item.node_name, associations))
        return result

    def list_nodes(self) -> List[str]:
        '''Node names of all loaded configurations.'''
        return [item.node_name
                for _cfgid, lc in self._snapshot_configs()
                for item in lc.nodes()]

    def ros_run(self, request_json: RosRun, return_as_json: bool = True) -> RosRunReply:
        Log.info(f"{self.__class__.__name__}: Request to [ros.launch.ros_run]: {request_json}")

        # Covert input dictionary into a proper python object
        request = request_json
        package = request.package
        binary = request.binary
        result = RosRunReply(package=package, binary=binary)
        try:
            ns = getattr(request, "ns", "")
            name = getattr(request, "name", binary)
            ns_name = ns_join(ns, name)
            screen_prefix = screen.get_cmd(ns_name)
            # environment with already normalized DISPLAY
            new_env = self._local_env()

            launch_prefix = getattr(request, "prefix", "")

            args = ' '.join(getattr(request, "args", []))
            # params
            ros_args = ''
            request_name = getattr(request, "name", "")
            request_ros_args = getattr(request, "ros_args", [])
            if ns or request_name or request_ros_args:
                ros_args = '--ros-args'
                if ns:
                    if not ns.startswith("/"):
                        ns = f"/{ns}"
                    ros_args += f" -r __ns:={ns}"
                if request_name:
                    ros_args += f" -r __name:={request_name}"
                if request_ros_args:
                    ros_args += f" {' '.join(request_ros_args)}"
            # start
            cmd = ' '.join([screen_prefix, launch_prefix, 'ros2', 'run', package, binary, args, ros_args])
            Log.info(f"{cmd}")
            Log.debug(f"environment while run node '{ns_name}': '{new_env}'")
            sp = SupervisedPopen(cmd, shell=True, env=new_env,
                                 object_id=f"run_node_{ns_name}", description=f"ros2 run [{package}]{binary}")
            result.result = True
            error = sp.stderr.read()
            if error:
                result.result = False
                result.message = error
        except exceptions.ResourceNotFound as err_nf:
            result.result = False
            result.message = f"Error while start binary '{binary}' from '{package}': {err_nf}"
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        except Exception:
            result.result = False
            result.message = f"Error while start binary '{binary}' from '{package}' {traceback.format_exc()}"
            Log.warn(f"{self.__class__.__name__}: {result.message}")
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        finally:
            nmd.launcher.server.screen_servicer.system_change()
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result

    def start_node(self, request_json: LaunchNode, return_as_json: bool = True) -> LaunchNodeReply:
        Log.info(f"{self.__class__.__name__}: Request to [ros.launch.start_node]")
        request = request_json
        result = LaunchNodeReply(name=request.name)
        daemonuri = getattr(request, 'masteruri', '') or ''
        try:
            launch_configs = []
            if request.opt_launch:
                launch_config = self._get_config(request.opt_launch, daemonuri)
                if launch_config is not None:
                    launch_configs.append(launch_config)
            if not launch_configs:
                for _cfgid, launchcfg in self._snapshot_configs():
                    if launchcfg.get_node(request.name) is not None:
                        Log.debug(f"Found launch file={launchcfg.filename};")
                        launch_configs.append(launchcfg)
            if not launch_configs:
                result.status.code = 'NODE_NOT_FOUND'
                result.status.msg = f"Node '{request.name}' not found"
                return self._reply(result, return_as_json)
            if len(launch_configs) > 1:
                result.status.code = 'MULTIPLE_LAUNCHES'
                result.status.msg = f"Node '{request.name}' found in multiple launch files"
                result.launch_files.extend([lcfg.filename for lcfg in launch_configs])
                return self._reply(result, return_as_json)
            try:
                result.launch_files.append(launch_configs[0].filename)
                # the callback also reports executables of delayed (TimerAction) nodes,
                # the return value is only used for the immediate start
                start_info = launch_configs[0].run_node(
                    request.name,
                    ignore_timer=getattr(request, "ignore_timer", False),
                    executable_callback=self._track_node_executable)
                if start_info and not os.path.exists(start_info):
                    # delayed start: run_node() returned an informational message
                    result.status.msg = start_info
                Log.debug(f'Node={request.name}; start finished')
                result.status.code = 'OK'
            except exceptions.BinarySelectionRequest as bsr:
                result.status.code = 'MULTIPLE_BINARIES'
                result.status.msg = (f"multiple binaries found for node "
                                     f"'{request.name}': {bsr.choices}")
                result.paths.extend(bsr.choices)
        except exceptions.ResourceNotFound as err_nf:
            result.status.code = 'ERROR'
            result.status.msg = f"Error while start node '{request.name}': {err_nf}"
        except Exception:
            result.status.code = 'ERROR'
            result.status.msg = (f"Error while start node '{request.name}': "
                                 f"{traceback.format_exc()}")
            Log.warn(f"{self.__class__.__name__}: {result.status.msg}")
        finally:
            # no return here: it would swallow exceptions of the except blocks
            nmd.launcher.server.screen_servicer.system_change()
        return self._reply(result, return_as_json)

    def _reply(self, result, return_as_json: bool):
        """Serialize a reply object if the caller requested JSON."""
        return json.dumps(result, cls=SelfEncoder) if return_as_json else result

    def _track_node_executable(self, node_name: str, executable_path: str) -> None:
        '''
        Observe the binary of a started node.

        add_file() is called at most once per node name, so node_stopped() can
        release exactly one observer reference.
        '''
        with self._node_exec_lock:
            if node_name in self._node_exec:
                return
            self._node_exec[node_name] = executable_path

        try:
            self._observer.add_file(executable_path)
        except Exception as error:
            # Without a watch the entry must not stay, otherwise node_stopped()
            # would release a reference that was never acquired.
            with self._node_exec_lock:
                self._node_exec.pop(node_name, None)
            Log.debug(f"{self.__class__.__name__}: cannot observe executable "
                      f"{executable_path} of node {node_name}: {error}")

    def node_stopped(self, node_name: str) -> None:
        '''
        Release the watch of a stopped node. Idempotent.

        The observer keeps a reference count per path, so remove_file() has to be
        called once for every add_file() call. Skipping the call while another
        node uses the same binary would leak the watch forever.
        '''
        with self._node_exec_lock:
            exec_path = self._node_exec.pop(node_name, None)

        if exec_path is None:
            return

        self._observer.remove_file(exec_path)

    def reconcile_running_nodes(self, alive_node_names: set) -> None:
        '''Release watches of nodes that are gone (crashed or killed externally).'''
        with self._node_exec_lock:
            tracked = set(self._node_exec.keys())
        for node_name in tracked - alive_node_names:
            try:
                self.node_stopped(node_name)
            except Exception:
                Log.warn(f"{self.__class__.__name__}: reconcile: cleanup for "
                         f"'{node_name}' failed:\n{traceback.format_exc()}")

    def start_nodes(self, request_json: List[LaunchNode], continue_on_error: bool = True) -> List[LaunchNodeReply]:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.launch.start_nodes]")

        result = []
        for request in request_json:
            node_result = self.start_node(request, return_as_json=False)
            result.append(node_result)
            if not continue_on_error:
                if node_result.status.code != 'OK':
                    break

        return json.dumps(result, cls=SelfEncoder)

    def get_included_files(self, request_json: LaunchIncludedFilesRequest, *,
                           result_as_json=True,
                           daemonuri: str = '') -> List[LaunchIncludedFile]:
        # Convert input dictionary into a proper python object
        request = request_json
        try:
            Log.info(
                f"{self.__class__.__name__}: Request to [ros.launch.get_included_files]: Path [{request.path}], args: {', '.join(f'{a.name}: {a.value}' for a in request.args)}")
        except Exception:
            Log.info(
                f"{self.__class__.__name__}: Request to [ros.launch.get_included_files]: Path [{request.path}], args: {request.args}")

        result = []
        cfg_included_files = []
        # The same launch file can be loaded for different daemon URIs, so the
        # configuration has to be looked up with the requested URI.
        cfg = self._get_config(request.path, daemonuri)
        if cfg is not None:
            if cfg.launch_type == 'python':
                return cfg._included_files
            cfg_included_files.extend(cfg._included_files)

        # This part is executed if the launch file is not loaded or is of type XML
        # TODO add parser for python launch files
        try:
            search_in_ext = SEARCH_IN_EXT
            if request.search_in_ext:
                search_in_ext = request.search_in_ext
            # search for loaded file and get the arguments
            resolve_args = {arg.name: arg.value for arg in request.args if hasattr(arg, "value")}
            if not resolve_args:
                for cfgid, lcfg in self._snapshot_configs():
                    if cfgid.path == request.path:
                        resolve_args.update(lcfg.resolve_dict)
                        break
            # replay each file
            for inc_file in xml.find_included_files(request.path, recursive=request.recursive, unique=request.unique, search_in_ext=search_in_ext, resolve_args=resolve_args):
                file_size = 0
                if inc_file.exists:
                    file_size = os.path.getsize(inc_file.inc_path)
                args = [LaunchArgument(name=name, value=value) for name, value in inc_file.args.items()]
                default_inc_args = [LaunchArgument(name=name, value=value) for name, value in inc_file.args.items()]
                org_inc_path = None
                if cfg_included_files:
                    # use resolved launch arguments from loaded configuration
                    # remove if used: case if the same launch files was loaded multiple times with different arguments
                    if cfg_included_files[0].path == inc_file.path_or_str:
                        if cfg_included_files[0].inc_path == inc_file.inc_path or cfg_included_files[0].line_number == inc_file.line_number:
                            args = cfg_included_files[0].args
                            default_inc_args = cfg_included_files[0].default_inc_args
                            org_inc_path = cfg_included_files[0].inc_path
                            del cfg_included_files[0]
                lincf = LaunchIncludedFile(path=inc_file.path_or_str,
                                           line_number=inc_file.line_number,
                                           inc_path=org_inc_path if org_inc_path is not None else inc_file.inc_path,
                                           inc_realpath=os.path.realpath(inc_file.inc_path),
                                           exists=inc_file.exists,
                                           raw_inc_path=inc_file.raw_inc_path,
                                           rec_depth=inc_file.rec_depth,
                                           args=args,
                                           default_inc_args=default_inc_args,
                                           size=file_size
                                           )
                result.append(lincf)
        except Exception:
            Log.warn(
                f"{self.__class__.__name__}: Can't get include files for {request.path}: {traceback.format_exc()}")
        return json.dumps(result, cls=SelfEncoder) if result_as_json else result

    def interpret_path(self, request_json: LaunchInterpretPathRequest) -> List[LaunchInterpretPathReply]:
        # Covert input dictionary into a proper python object
        request = request_json
        text = request.text
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.interpret_path]: {text}")
        args = {arg.name: arg.value for arg in request.args if hasattr(arg, "value")}
        result = []
        if text:
            try:
                for inc_file in xml.find_included_files(text, False, False, search_in_ext=[]):
                    aval = inc_file.raw_inc_path
                    aitems = aval.split("'")
                    for search_for in aitems:
                        if not search_for:
                            continue
                        Log.debug(
                            f"{self.__class__.__name__}: try to interpret: {search_for}")
                        args_in_name = xml.get_arg_names(search_for)
                        request_args = False
                        for arg_name in args_in_name:
                            if arg_name not in args:
                                request_args = True
                                break
                        if request_args:
                            req_args = []
                            for arg_name in args_in_name:
                                if arg_name in args:
                                    req_args.append(LaunchArgument(
                                        arg_name, args[arg_name]))
                                else:
                                    req_args.append(LaunchArgument(arg_name, ""))
                            reply = LaunchInterpretPathReply(
                                text=search_for, status='PARAMS_REQUIRED', args=req_args)
                            reply.status.code = 'PARAMS_REQUIRED'
                            result.append(reply)
                        else:
                            search_for_rpl = xml.replace_arg(search_for, args)
                            reply = LaunchInterpretPathReply(
                                text=search_for, status='OK', path=search_for_rpl, exists=os.path.exists(search_for), args=request.args)
                            result.append(reply)
            except Exception as err:
                reply = LaunchInterpretPathReply(
                    text=text, status='ERROR', args=request.args)
                reply.status.msg = err
                result.append(reply)
        else:
            reply = LaunchInterpretPathReply(
                text=text, status='ERROR', args=request.args)
            reply.status.msg = 'empty request'
            result.append(reply)
        return json.dumps(result, cls=SelfEncoder)

    def get_msg_struct(self, msg_type: str) -> LaunchMessageStruct:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.get_msg_struct]: "
                  f"msg [{msg_type}]")
        result = LaunchMessageStruct(msg_type)
        try:
            if self._is_action_type(msg_type):
                _action_class, msg_class = self._get_action_types(msg_type)
            else:
                msg_class = self._message_class(msg_type)
            if not hasattr(msg_class, 'get_fields_and_field_types'):
                result.message = (f"unexpected message class: '{msg_class}', no "
                                  f"'get_fields_and_field_types' attribute found!")
                return json.dumps(result, cls=SelfEncoder)
            # the expansion is deterministic for a type: cache it
            definition = MESSAGE_STRUCT_CACHE.get_or_create(
                'msg', msg_type,
                lambda: self._expand_fields(msg_class.get_fields_and_field_types()))
            result.data = {'type': msg_type, 'name': '', 'def': definition}
            result.valid = True
        except Exception as err:
            print(traceback.format_exc())
            result.message = repr(err)
            result.valid = False
        return json.dumps(result, cls=SelfEncoder)

    @staticmethod
    def _parse_field_type(field_type: str) -> Tuple[str, bool, Optional[str]]:
        '''Split a field type into base type, array flag and sequence length.'''
        if field_type.startswith('sequence'):
            # sequences defined with sequence<>
            match = SEQUENCE_TYPE_RE.search(field_type)
            if match is not None:
                return match.group(1), True, match.group(2)
            return field_type, True, None
        if '[' in field_type:
            # arrays defined with []
            match = ARRAY_TYPE_RE.search(field_type)
            if match is not None:
                return match.group(1), True, match.group(2)
            return field_type, True, None
        return field_type, False, None

    def _struct_for_type(self, base_type: str) -> List[Dict]:
        '''Expanded definition of a nested message type, cached per type.'''
        def factory() -> List[Dict]:
            # Field types are reported as 'pkg/Type' or 'pkg/msg/Type'.
            parts = base_type.split('/')
            package_name = parts[0]
            type_name = parts[-1]
            module = import_module('.msg', package_name)
            msg_class = getattr(module, type_name, None)
            if msg_class is None:
                raise LookupError(f"unknown nested type: {base_type}")
            return self._expand_fields(msg_class.get_fields_and_field_types())

        try:
            return MESSAGE_STRUCT_CACHE.get_or_create('nested', base_type, factory)
        except (ImportError, LookupError) as error:
            # Report the field as a leaf instead of failing and do not cache the
            # negative result: the type may become importable later.
            Log.debug(f"{self.__class__.__name__}: cannot expand nested type "
                      f"{base_type}: {error}")
            return []

    # create recursive dictionary for 'ros.launch.get_msg_struct'
    def _expand_fields(self, field_and_types: Dict[str, str]) -> List[Dict]:
        defs = []
        for field_name, field_type in field_and_types.items():
            base_type, is_array, seq_length = self._parse_field_type(field_type)
            field_struct = {'name': field_name, 'def': []}
            if base_type not in [*rosidl_parser.definition.BASIC_TYPES,
                                 'string', 'str', 'wstring']:
                # complex type: expansion is cached per type name
                field_struct['def'] = self._struct_for_type(base_type)
            reported_type = base_type
            if is_array:
                reported_type += f'[{seq_length}]' if seq_length else '[]'
            field_struct['type'] = reported_type
            field_struct['is_array'] = is_array
            if seq_length:
                field_struct['length'] = seq_length
            defs.append(field_struct)
        return defs

    @staticmethod
    def str2typedValue(value, value_type):
        if value_type.startswith('bool'):
            try:
                return value.lower() in ("yes", "true", "t", "y", "1")
            except AttributeError:
                return value
        for marker, converter in _TYPE_CONVERTERS:
            if marker in value_type:
                try:
                    return converter(value)
                except (TypeError, ValueError):
                    return value
        return value

    def _str_from_dict(self, param_dict):
        result = dict()
        fields = param_dict if isinstance(param_dict, list) else param_dict['def']
        for field in fields:
            if not field['def']:
                # simple types
                if 'value' in field and field['value']:
                    base_type = field['type'].replace(r'/\[\d*\]/', '')
                    if field['is_array']:
                        # parse to array
                        listvals = [a.strip() for a in list(csv.reader([field['value'].replace(', "', ',"')]))[0]]
                        result[field['name']] = [self.str2typedValue(
                            n, base_type) for n in listvals]
                    else:
                        result[field['name']] = self.str2typedValue(
                            field['value'], base_type)
            elif field['is_array']:
                # TODO: create array for base types
                result_array = []
                # it is a complex field type
                if 'value' in field:
                    for array_element in field['value']:
                        result_array.append(
                            self._str_from_dict(array_element))
                # append created array
                if result_array:
                    result[field['name']] = result_array
            else:
                if 'useNow' in field and os.environ.get('ROS_DISTRO') != 'galactic' and field['useNow']:
                    result[field['name']] = "now"
                else:
                    sub_result = self._str_from_dict(field['def'])
                    if sub_result:
                        result[field['name']] = sub_result
        return result

    def publish_message(self, request_json: LaunchPublishMessage) -> str:
        result = None
        try:
            # Convert input dictionary into a proper python object
            request = request_json
            Log.debug(
                f"{self.__class__.__name__}: Request to [ros.launch.publish_message]: msg [{request.msg_type}]")
            opt_str = ''
            if request.once:
                opt_str = '-1'
            elif request.latched:
                # quality of service for latched topics
                opt_str = '--qos-durability transient_local --qos-reliability reliable'
            elif request.rate != 0.0:
                opt_str = f"-r {request.rate}"
            if request.verbose:
                opt_str += ' -p 1'
            else:
                opt_str += ' -p 10'
            if request.use_rostime:
                opt_str += ' --use-sim-time'
            ns, name = ros2_publisher_nodename_tuple(request.topic_name)
            fullname = os.path.join(ns, name).replace('/', '_')
            opt_str += f' -n {fullname}'
            data = json.loads(request.data)
            topic_params = self._str_from_dict(data)
            qos_params = ""
            if hasattr(request, "qos"):
                default_qos = RosQos()
                if hasattr(request.qos, "durability") and request.qos.durability != default_qos.durability:
                    qos_params += f"--qos-durability {RosQos.durabilityToString(request.qos.durability)} "
                if hasattr(request.qos, "reliability") and request.qos.reliability != default_qos.reliability:
                    qos_params += f"--qos-reliability {RosQos.reliabilityToString(request.qos.reliability)} "
                if hasattr(request.qos, "liveliness") and request.qos.liveliness != default_qos.liveliness:
                    qos_params += f"--qos-liveliness {RosQos.livelinessToString(request.qos.liveliness)} "
                if hasattr(request.qos, "history") and request.qos.history != default_qos.history and request.qos.history < RosQos.HISTORY.UNKNOWN:
                    qos_params += f"--qos-history {RosQos.historyToString(request.qos.history)} "
                if hasattr(request.qos, "depth") and request.qos.depth != default_qos.depth:
                    qos_params += f"--qos-depth {request.qos.depth} "
                if hasattr(request.qos, "liveliness_lease_duration"):
                    if hasattr(request.qos.liveliness_lease_duration, "sec") and request.qos.liveliness_lease_duration.sec != default_qos.liveliness_lease_duration.sec:
                        qos_params += f"--qos-liveliness-lease-duration-seconds {request.qos.liveliness_lease_duration} "
            pub_cmd = f"pub {opt_str} {qos_params} {request.topic_name} {request.msg_type} \"{topic_params}\""
            screen_prefix = screen.get_cmd(fullname)
            cmd = ' '.join([screen_prefix, 'ros2', 'topic', pub_cmd])
            Log.info(
                f"{self.__class__.__name__}: run ros2 publisher with: {cmd}")
            SupervisedPopen(shlex.split(cmd),
                            object_id=f"ros_topic_pub_{request.topic_name}", description=f"publish to topic {request.topic_name}")
            result = {"result": True, "message": ""}
        except Exception:
            error_msg = traceback.format_exc()
            print(error_msg)
            result = {"result": False, "message": error_msg}
        return json.dumps(result, cls=SelfEncoder)

    @staticmethod
    def _is_action_type(identifier: str) -> bool:
        '''Whether the identifier belongs to an action interface.'''
        return identifier.find("/action/") != -1

    @staticmethod
    @lru_cache(maxsize=256)
    def _message_class(type_name: str):
        '''Cached interface lookup, the import is expensive.'''
        return get_message(type_name)

    @staticmethod
    @lru_cache(maxsize=256)
    def _service_class(type_name: str):
        '''Cached service interface lookup.'''
        return get_service(type_name)

    @staticmethod
    @lru_cache(maxsize=256)
    def _get_action_types(identifier: str) -> Tuple[ActionClass, ActionRequestClass]:
        '''
        Cached lookup of the action class and the class of the requested part.

        Must not be an instance method: lru_cache would keep a strong reference
        to the servicer instance in a class wide cache.
        '''
        is_action_result = False
        is_action_goal = False
        is_action_feedback = False
        normalized_identifier = identifier
        if identifier.endswith("_GetResult"):
            is_action_result = True
            normalized_identifier = identifier.replace("_GetResult", "")
        elif identifier.endswith("_SendGoal"):
            is_action_goal = True
            normalized_identifier = identifier.replace("_SendGoal", "")
        elif identifier.endswith("_Goal"):
            is_action_goal = True
            normalized_identifier = identifier.replace("_Goal", "")
        elif identifier.endswith("_FeedbackMessage"):
            is_action_feedback = True
            normalized_identifier = identifier.replace("_FeedbackMessage", "")
        # cancel goal is of type srv not action :/
        if identifier.find("/srv/CancelGoal") != -1:
            action_class = get_service(normalized_identifier)
        else:
            action_class = get_action(normalized_identifier)

        if is_action_result:
            request_class = action_class.Result
        elif is_action_goal:
            request_class = action_class.Goal
        elif is_action_feedback:
            request_class = action_class.Feedback
        else:
            request_class = action_class
        return action_class, request_class

    def get_srv_struct(self, srv_type: str) -> LaunchMessageStruct:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.get_srv_struct]: "
                  f"srv [{srv_type}]")
        result = LaunchMessageStruct(srv_type)
        try:
            if self._is_action_type(srv_type):
                _action_class, request_class = self._get_action_types(srv_type)
            else:
                request_class = self._service_class(srv_type).Request
            if not hasattr(request_class, 'get_fields_and_field_types'):
                result.message = (f"unexpected service class: '{request_class}', no "
                                  f"'get_fields_and_field_types' attribute found!")
                return json.dumps(result, cls=SelfEncoder)
            definition = MESSAGE_STRUCT_CACHE.get_or_create(
                'srv', srv_type,
                lambda: self._expand_fields(request_class.get_fields_and_field_types()))
            result.data = {'type': srv_type, 'name': '', 'def': definition}
            result.valid = True
        except Exception as err:
            print(traceback.format_exc())
            result.message = repr(err)
            result.valid = False
        return json.dumps(result, cls=SelfEncoder)

    def _destroy_service_clients(self, service_name: str) -> None:
        '''Destroy all clients created for this service name.'''
        try:
            for client in list(nmd.ros_node.clients):
                if client.srv_name == service_name:
                    nmd.ros_node.destroy_client(client)
        except Exception:
            Log.debug(
                f"{self.__class__.__name__}: cleanup of clients for '{service_name}' failed:\n{traceback.format_exc()}")

    def call_service(self, request_json: LaunchCallService) -> None:
        # Convert input dictionary into a proper python object
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.call_service]: {request_json}")
        request = request_json
        result = LaunchMessageStruct(request.srv_type)
        result.valid = False
        is_action = self._is_action_type(request.srv_type) or "/_action/" in request.service_name
        try:
            data = json.loads(request.data) if getattr(request, "data", "") else {}
            fields = self._str_from_dict(data) if data else {}
            if is_action:
                # an action looks like a service, but needs a special handling
                action_class, request_class = self._get_action_types(request.srv_type)
                service_request = request_class()
                if fields:
                    set_message_fields(service_request, fields)
                response = nmd.launcher.call_action(request.service_name, action_class, service_request, 10)
            else:
                # cached lookup, the interface import is expensive
                request_class = self._service_class(request.srv_type)
                service_request = request_class.Request()
                if fields:
                    set_message_fields(service_request, fields)
                response = nmd.launcher.call_service(
                    request.service_name, request_class, service_request,
                    timeout_sec=10, callback_group=self._callback_service_group)
            if response is not None:
                result.data = message_to_ordereddict(response)
                result.valid = True
            else:
                result.message = 'did not receive a reply'
        except Exception as e:
            result.message = f"Exception while calling service: {e!r}"
            Log.warn(f"{self.__class__.__name__}: {result.message}\n{traceback.format_exc()}")
        finally:
            if not is_action:
                # action clients are owned and cleaned up by nmd.launcher.call_action
                self._destroy_service_clients(request.service_name)
        return json.dumps(result, cls=SelfEncoder)

    def get_message_types(self, mode: str = "message") -> str:
        Log.debug("Request to [ros.launch.get_message_types]")
        result = []
        interfaces = {}
        if mode == "service":
            interfaces = get_service_interfaces()
        elif mode == "action":
            interfaces = get_action_interfaces()
        else:
            interfaces = get_message_interfaces()

        for pkg, messages in interfaces.items():
            for message in messages:
                result.append(f"{pkg}/{message}")
        return json.dumps(result, cls=SelfEncoder)

    def _start_mas_node(self, executable: str, fullname: str, args: List[str]) -> str:
        '''Start one of the mas helper nodes inside a screen session.
        All state is local, the method is safe for concurrent requests.'''
        package_name = 'fkie_mas_daemon'
        cmd = f"ros2 run {package_name} {executable}"
        screen_prefix = screen.get_cmd(fullname)
        new_env = self._local_env()
        Log.info(f"{self.__class__.__name__}: {screen_prefix} {cmd} {' '.join(args)}")
        SupervisedPopen(shlex.split(' '.join([screen_prefix, cmd] + args)),
                        env=new_env,
                        object_id=f"run_node_{fullname}",
                        description=f"Run [{package_name}]{executable}")
        return json.dumps({"result": True, "message": ""}, cls=SelfEncoder)

    def start_subscriber(self, request_json: SubscriberNode) -> str:
        request = request_json
        topic = request.topic
        Log.debug(f"{self.__class__.__name__}: Request to [ros.subscriber.start]: {topic}")
        namespace, name = ros2_subscriber_nodename_tuple(topic)
        fullname = os.path.join(namespace, name)
        args = [f'--ws_port={self.ws_port}',
                f'--topic={topic}',
                f'--message_type={request.message_type}']
        if request.filter.no_data:
            args.append('--no_data')
        if request.filter.no_arr:
            args.append('--no_arr')
        if request.filter.no_str:
            args.append('--no_str')
        args.append(f'--hz={request.filter.hz}')
        args.append(f'--window={request.filter.window}')
        if hasattr(request.filter, "arrayItemsCount"):
            args.append(f'--array_items_count={request.filter.arrayItemsCount}')
        if request.tcp_no_delay:
            args.append('--tcp_no_delay')
        if getattr(request, "qos", None):
            if request.qos.durability:
                args.append(f'--qos-durability={RosQos.durabilityToString(request.qos.durability)}')
            if request.qos.reliability:
                args.append(f'--qos-reliability={RosQos.reliabilityToString(request.qos.reliability)}')
            if request.qos.liveliness:
                args.append(f'--qos-liveliness={RosQos.livelinessToString(request.qos.liveliness)}')
        else:
            # TODO wait for publisher and detect qos
            pass
        return self._start_mas_node('mas-subscriber', fullname, args)

    def start_action(self, request_json) -> str:
        '''Start an action client node to send a goal to a ROS action server.'''
        request = request_json
        Log.debug(f"{self.__class__.__name__}: Request to [ros.action.send_goal]: {request}")
        namespace, name = ros2_action_nodename_tuple(request.action_name)
        fullname = os.path.join(namespace, name)
        args = [f'--ws_port={self.ws_port}',
                f'--action_name={request.action_name}',
                f'--action_type={request.action_type}']
        if request.goal:
            args.append(f"--goal_json='{request.goal}'")
        return self._start_mas_node('mas-action-client', fullname, args)

    def start_action_introspection(self, request_json) -> str:
        '''Start a node that subscribes to the *_service_event introspection
        topics of a ROS action and forwards events via websocket.'''
        request = request_json
        Log.debug(f"{self.__class__.__name__}: Request to "
                  f"[ros.action.introspection.start]: {request}")
        # own node name, so it does not collide with the action client
        namespace, name = ros2_action_introspection_nodename_tuple(request.action_name)
        fullname = os.path.join(namespace, name)
        args = [f'--ws_port={self.ws_port}',
                f'--action_name={request.action_name}',
                f'--action_type={request.action_type}']
        return self._start_mas_node('mas-action-introspection', fullname, args)

    def start_service_introspection(self, request_json) -> str:
        request = request_json
        namespace, name = ros2_service_introspection_nodename_tuple(request.service_name)
        fullname = os.path.join(namespace, name)
        args = [f'--ws_port={self.ws_port}',
                f'--service_name={request.service_name}',
                f'--service_type={request.service_type}']
        return self._start_mas_node('mas-service-introspection', fullname, args)
