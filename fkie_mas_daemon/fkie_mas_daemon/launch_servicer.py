# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import fkie_mas_daemon as nmd
from .launch_validator import LaunchValidator
from .launch_context import LaunchContext
from .launch_config import LaunchConfig
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
from fkie_mas_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_mas_pylib.interface.launch_interface import LaunchPublishMessage
from fkie_mas_pylib.interface.launch_interface import LaunchMessageStruct
from fkie_mas_pylib.interface.launch_interface import LaunchIncludedFile
from fkie_mas_pylib.interface.launch_interface import LaunchIncludedFilesRequest
from fkie_mas_pylib.interface.launch_interface import LaunchInterpretPathReply
from fkie_mas_pylib.interface.launch_interface import LaunchInterpretPathRequest
from fkie_mas_pylib.interface.launch_interface import LaunchNodeReply
from fkie_mas_pylib.interface.launch_interface import LaunchNodeInfo
from fkie_mas_pylib.interface.launch_interface import LaunchNode
from fkie_mas_pylib.interface.launch_interface import LaunchAssociations
from fkie_mas_pylib.interface.launch_interface import LaunchContent
from fkie_mas_pylib.interface.launch_interface import LaunchLoadReply
from fkie_mas_pylib.interface.launch_interface import LaunchLoadRequest
from fkie_mas_pylib.interface.launch_interface import LaunchFile
from fkie_mas_pylib.interface.launch_interface import LaunchCallService
from fkie_mas_pylib.interface.launch_interface import LaunchArgument
from fkie_mas_pylib.interface.runtime_interface import RosQos
from fkie_mas_pylib.interface.runtime_interface import SubscriberNode
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib import ros_pkg
from rosidl_runtime_py.utilities import is_service
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.utilities import get_service
from rosidl_runtime_py.utilities import get_interface
from rosidl_runtime_py.utilities import get_action
from rosidl_runtime_py import get_service_interfaces
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import get_action_interfaces
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py import message_to_ordereddict
import rosidl_parser.definition
from watchdog.events import FileSystemEvent
from watchdog.events import LoggingEventHandler
from watchdog.observers import Observer
import csv
import json
import os
import re
import shlex
import sys
import traceback
from importlib import import_module
# from launch.launch_context import LaunchContext

from typing import Dict
from typing import List
from typing import Tuple
from typing import Type
ActionClass = Type
ActionRequestClass = Type


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


class LaunchServicer(LoggingEventHandler):
    '''
    '''

    def __init__(self, websocket: WebSocketServer, ws_port: int):
        Log.info("Create ROS2 launch servicer")
        LoggingEventHandler.__init__(self)
        self._watchdog_observer = Observer()
        self.ws_port = ws_port
        self.xml_validator = LaunchValidator()
        self._observed_dirs = {}  # path: watchdog.observers.api.ObservedWatch
        self._real_paths = {}  # link <-> real path; real path <-> real path
        self._node_exec = {}  # node name <-> executable path
        self._included_files = []
        self._included_dirs = []
        self._is_running = True
        self._peers = {}
        self._loaded_files: Dict[CfgId, LaunchConfig] = dict()
        self._observed_launch_files: Dict[str, List[str]] = {}
        self._watchdog_observer.start()
        self.websocket = websocket
        websocket.register("ros.launch.load", self.load_launch)
        websocket.register("ros.launch.reload", self.reload_launch)
        websocket.register("ros.launch.unload", self.unload_launch)
        websocket.register("ros.launch.get_list", self.get_list)
        websocket.register("ros.launch.start_node", self.start_node)
        websocket.register("ros.launch.start_nodes", self.start_nodes)
        websocket.register("ros.launch.get_included_files", self.get_included_files)
        websocket.register("ros.launch.interpret_path", self.interpret_path)
        websocket.register("ros.launch.get_msg_struct", self.get_msg_struct)
        websocket.register("ros.launch.publish_message", self.publish_message)
        websocket.register("ros.launch.get_srv_struct", self.get_srv_struct)
        websocket.register("ros.launch.call_service", self.call_service)
        websocket.register("ros.launch.get_message_types", self.get_message_types)
        websocket.register("ros.subscriber.start", self.start_subscriber)

    def _terminated(self):
        Log.info(f"{self.__class__.__name__}: terminated launch context")

    def _register_callback(self, context):
        if (context.peer() not in self._peers):
            Log.info(
                f"{self.__class__.__name__}: Add callback to peer context @{context.peer()}")
            if context.add_callback(self._terminated):
                self._peers[context.peer()] = context

    def stop(self):
        '''
        Stop watchdog and cancel the autostart of the nodes.
        '''
        self._is_running = False
        self._watchdog_observer.stop()

    def on_any_event(self, event: FileSystemEvent):
        if event.event_type in ['opened', 'closed']:
            return
        path = ''
        if event.src_path in self._real_paths:
            path = self._real_paths[event.src_path]
        if path in self._included_files:
            affected_launch_files = []
            for launch_path, path_list in self._observed_launch_files.items():
                if path in path_list:
                    affected_launch_files.append(launch_path)
            change_event = {"eventType": event.event_type,
                            "srcPath": path,
                            "affected": affected_launch_files}
            Log.debug(
                f"{self.__class__.__name__}: observed change {event.event_type} on {event.src_path}, reported path: {path}")
            self.websocket.publish('ros.path.changed', change_event)

    def _add_file_to_observe(self, path: str):
        real_path = os.path.realpath(path)
        self._real_paths[real_path] = path
        directory = os.path.dirname(real_path)
        Log.debug(f"{self.__class__.__name__}:observe path: {path}")
        if directory not in self._observed_dirs:
            Log.debug(
                f"{self.__class__.__name__}: add directory to observer: {directory}")
            watch = self._watchdog_observer.schedule(self, directory)
            self._observed_dirs[directory] = watch
        self._included_files.append(path)
        self._included_dirs.append(directory)

    def _remove_file_from_observe(self, path: str):
        Log.debug(f"{self.__class__.__name__}: stop observe path: {path}")
        try:
            real_path = os.path.realpath(path)
            directory = os.path.dirname(real_path)
            self._included_files.remove(path)
            if path not in self._included_files:
                del self._real_paths[real_path]
            self._included_dirs.remove(directory)
            if directory not in self._included_dirs:
                if directory in self._observed_dirs:
                    Log.debug(
                        f"{self.__class__.__name__}: remove directory from observer: {directory}")
                    self._watchdog_observer.unschedule(
                        self._observed_dirs[directory])
                    del self._observed_dirs[directory]
        except ValueError:
            pass

    def _add_launch_to_observer(self, launch_config: LaunchConfig):
        added = []
        try:
            self._add_file_to_observe(launch_config.filename)
            added.append(launch_config.filename)
            for inc_discription in launch_config._included_files:
                self._add_file_to_observe(inc_discription.inc_path)
                added.append(inc_discription.inc_path)
        except Exception as e:
            Log.error(
                f"{self.__class__.__name__}: _add_launch_to_observer {launch_config.filename}: \n {e}")
        self._observed_launch_files[launch_config.filename] = added

    def _remove_launch_from_observer(self, launch_config: LaunchConfig):
        try:
            for path in self._observed_launch_files[launch_config.filename]:
                try:
                    self._remove_file_from_observe(path)
                except Exception as e:
                    Log.error(
                        f"{self.__class__.__name__}: _remove_file_from_observe {path}:\n{e}")
            del self._observed_launch_files[launch_config.filename]
        except Exception as e:
            Log.error(
                f"{self.__class__.__name__}: _remove_launch_from_observer {launch_config.filename}:\n{e}")

    # def start_node(self, node_name):
    #     global IS_RUNNING
    #     if not IS_RUNNING:
    #         return
    #     for _cfgid, launchcfg in self._loaded_files.items():
    #         n = launchcfg.get_node(node_name)
    #         if n is not None:
    #             startcfg = launcher.create_start_config(
    #                 node_name, launchcfg, '', daemonuri='', loglevel='')
    #             launcher.run_node(startcfg)
    #             return
    #     raise Exception("Node '%s' not found!" % node_name)

    # def _autostart_nodes_threaded(self, cfg):
    #     global IS_RUNNING
    #     for item in cfg.roscfg.nodes:
    #         if not IS_RUNNING:
    #             return
    #         node_fullname = ns_join(item.namespace, item.name)
    #         try:
    #             if self._get_start_exclude(cfg, node_fullname):
    #                 # skip autostart
    #                 nmd.ros_node.get_logger().debug(
    #                     "%s is in exclude list, skip autostart" % node_fullname)
    #                 continue
    #             self._autostart_node(node_fullname, cfg)
    #         except Exception as err:
    #             nmd.ros_node.get_logger().warn("Error while start %s: %s" % (node_fullname, err))

    # def _autostart_node(self, node_name, cfg):
    #     global IS_RUNNING
    #     if not IS_RUNNING:
    #         return
    #     start_required = self._get_start_required(cfg, node_name)
    #     start_now = False
    #     if start_required:
    #         import rosgraph
    #         # get published topics from ROS master
    #         master = rosgraph.masterapi.Master(cfg.masteruri)
    #         for topic, _datatype in master.getPublishedTopics(''):
    #             if start_required == topic:
    #                 start_now = True
    #                 break
    #         if not start_now:
    #             # Start the timer for waiting for the topic
    #             start_timer = threading.Timer(
    #                 3.0, self._autostart_node, args=(node_name, cfg))
    #             start_timer.start()
    #     else:
    #         start_now = True
    #     if start_now:
    #         startcfg = launcher.create_start_config(
    #             node_name, cfg, '', daemonuri='', loglevel='')
    #         start_delay = self._get_start_delay(cfg, node_name)
    #         if start_delay > 0:
    #             # start timer for delayed start
    #             start_timer = threading.Timer(
    #                 start_delay, launcher.run_node, args=(startcfg,))
    #             start_timer.setDaemon(True)
    #             start_timer.start()
    #         else:
    #             launcher.run_node(startcfg)

    def load_launch(self, request_json: LaunchLoadRequest, return_as_json: bool = True) -> LaunchLoadReply:
        '''
        Loads launch file by interface request
        '''
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.load]")
        result = LaunchLoadReply(paths=[], args=[], changed_nodes=[])

        # Covert input dictionary into a proper python object
        request = request_json

        launchfile = request.path
        daemonuri = ''
        if hasattr(request, 'masteruri'):
            daemonuri = request.masteruri
        Log.debug(f"{self.__class__.__name__}: Loading launch file: {launchfile} (package: {request.ros_package}, launch: {request.launch}), daemonuri: {daemonuri}, host: {request.host}, args: {request.args}")

        if not launchfile:
            # determine path from package name and launch name
            try:
                paths = ros_pkg.get_share_files_path_from_package(
                    request.ros_package, request.launch)
                if not paths:
                    result.status.code = 'FILE_NOT_FOUND'
                    result.status.error_msg = "Launch files %s in package %s not found!" % (
                        request.launch, request.ros_package)
                    return json.dumps(result, cls=SelfEncoder) if return_as_json else result
                elif len(paths) > 1:
                    if request.force_first_file:
                        launchfile = paths[0]
                    else:
                        result.status.code = 'MULTIPLE_LAUNCHES'
                        result.status.msg = "Multiple launch files with name %s in package %s found!" % (
                            request.launch, request.ros_package)
                        for mp in paths:
                            result.paths.append(mp)
                        Log.debug(
                            f"{self.__class__.__name__}: ..load aborted, MULTIPLE_LAUNCHES")
                        return json.dumps(result, cls=SelfEncoder) if return_as_json else result
                else:
                    launchfile = paths[0]
            except LookupError as rnf:
                result.status.code = 'FILE_NOT_FOUND'
                result.status.msg = "Package %s not found: %s" % (
                    request.ros_package, rnf)
                Log.debug(
                    f"{self.__class__.__name__}: ..load aborted, FILE_NOT_FOUND")
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        result.paths.append(launchfile)
        # it is already loaded?
        if (launchfile, daemonuri) in list(self._loaded_files.keys()):
            result.status.code = 'ALREADY_OPEN'
            result.status.msg = "Launch file %s already loaded!" % (
                launchfile)
            Log.debug(
                f"{self.__class__.__name__}: ..load aborted, ALREADY_OPEN")
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result

        # load launch configuration
        try:
            # validate xml
            self.xml_validator.validate(launchfile)
            # test for required args
            provided_arg_names = [arg.name for arg in request.args]
            # get the list with needed launch args
            launch_context = LaunchContext(argv=sys.argv[1:])
            provided_args = [] if request.args is None else request.args
            req_args = LaunchConfig.get_launch_arguments(launch_context, launchfile,  provided_args = None if request.request_args else provided_args)
            # req_args_dict = launch_config.argv2dict(req_args)
            if request.request_args and req_args:
                for arg in req_args:
                    if arg.name not in provided_arg_names:
                        result.args.extend(req_args)
                        result.status.code = 'PARAMS_REQUIRED'
                        Log.debug(
                            f"{self.__class__.__name__}: ..load aborted, PARAMS_REQUIRED {[arg.name for arg in result.args]}; provided args {provided_arg_names}")
                        return json.dumps(result, cls=SelfEncoder) if return_as_json else result
            launch_arguments = [(arg.name, arg.value) if hasattr(arg, "value") else (arg.name, arg.default_value)  for arg in req_args ]
            launch_config = LaunchConfig(
                launchfile, context=launch_context, daemonuri=daemonuri, launch_arguments=launch_arguments)
            Log.debug(f"{self.__class__.__name__}: daemonuri: {daemonuri}")
            self._loaded_files[CfgId(launchfile, daemonuri)] = launch_config
            # notify GUI about changes
            self.websocket.publish('ros.launch.changed', {
                                   'path': launchfile, 'action': 'loaded'})
            self._add_launch_to_observer(launch_config)
            Log.debug(f"{self.__class__.__name__}: ..load complete!")
        except Exception as e:
            import traceback
            print(traceback.format_exc())
            err_text = "%s loading failed!" % launchfile
            err_details = "%s: %s" % (err_text, e)
            Log.warn(
                f"{self.__class__.__name__}: Loading launch file: {err_details}")
            result.status.code = 'ERROR'
            result.status.msg = err_details
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        result.status.code = 'OK'
        return json.dumps(result, cls=SelfEncoder) if return_as_json else result

    def reload_launch(self, request_json: LaunchLoadRequest) -> LaunchLoadReply:
        '''
        Reloads launch file by interface request
        '''
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.reload]")
        result = LaunchLoadReply(paths=[], args=[], changed_nodes=[])

        # Covert input dictionary into a proper python object
        request = request_json

        daemonuri = ''
        if hasattr(request, 'masteruri'):
            daemonuri = request.masteruri
        Log.debug(f"{self.__class__.__name__}: Loading launch file: {request.path} (package: {request.ros_package}, launch: {request.launch}), daemonuri: {daemonuri}, host: {request.host}, args: {request.args}")

        result.paths.append(request.path)
        cfgid = CfgId(request.path, daemonuri)
        Log.debug(
            f"{self.__class__.__name__}: reload launch file: {request.path}, daemonuri: {daemonuri}")
        if cfgid in self._loaded_files:
            old_launch = self._loaded_files[cfgid]
            try:
                self._remove_launch_from_observer(old_launch)
                # use argv from already open file
                launch_context = LaunchContext(argv=sys.argv[1:])
                launch_config = LaunchConfig(
                    old_launch.filename, context=launch_context, daemonuri=daemonuri, launch_arguments=old_launch.provided_launch_arguments)
                self._loaded_files[cfgid] = launch_config
                result.status.code = 'OK'
                # change detection for nodes parameters
                old_nodes = old_launch.nodes()
                new_nodes = launch_config.nodes()
                nodes2start = []
                for new_node in new_nodes:
                    found = False
                    for old_node in old_nodes:
                        if new_node.node_name == old_node.node_name:
                            found = True
                            if new_node.env and old_node.env:
                                if len(set(new_node.env.values()) - set(old_node.env.values())) > 0:
                                    nodes2start.append(new_node.node_name)
                                    break
                            if new_node.launch_prefix != old_node.launch_prefix:
                                nodes2start.append(new_node.node_name)
                                break
                            if new_node.cmd != old_node.cmd:
                                new_matches = re.findall(r'--params-file\s+([^\s]+)', new_node.cmd)
                                old_matches = re.findall(r'--params-file\s+([^\s]+)', old_node.cmd)
                                if len(new_matches) != len(old_matches):
                                    nodes2start.append(new_node.node_name)
                                    break
                                else:
                                    normalized_new_cmd = new_node.cmd
                                    normalized_old_cmd = old_node.cmd
                                    added = False
                                    # we need to compare the content of the parameter files
                                    for a, b in zip(new_matches, old_matches):
                                        content1 = ""
                                        content2 = ""
                                        with open(a, 'r') as f1, open(b, 'r') as f2:
                                            content1 = f1.read()
                                            content2 = f2.read()
                                        if content1 != content2:
                                            nodes2start.append(new_node.node_name)
                                            added = True
                                            break
                                        else:
                                            normalized_new_cmd = normalized_new_cmd.replace(a, '')
                                            normalized_old_cmd = normalized_old_cmd.replace(b, '')
                                    if not added and normalized_new_cmd != normalized_old_cmd:
                                        nodes2start.append(new_node.node_name)
                                        break
                            if new_node.additional_env and old_node.additional_env:
                                if len(set(new_node.additional_env) - set(old_node.additional_env)) > 0:
                                    nodes2start.append(new_node.node_name)
                                    break
                    if not found:
                        nodes2start.append(new_node.node_name)
                # filter out anonymous nodes
                for n in nodes2start:
                    if not re.search(r"\d{3,6}_\d{10,}", n):
                        result.changed_nodes.append(n)                # notify GUI about changes
                old_launch.unload()
                self.websocket.publish('ros.launch.changed', {
                                       'path': request.path, 'action': 'reloaded'})
                self._add_launch_to_observer(launch_config)
            except Exception as e:
                old_launch.load()
                self._add_launch_to_observer(old_launch)
                print(traceback.format_exc())
                err_text = f"{request.path} loading failed!"
                err_details = f"{err_text}: {e}"
                Log.warn(
                    f"{self.__class__.__name__}: Loading launch file: {err_details}")
                result.status.code = 'ERROR'
                result.status.msg = err_details
                return json.dumps(result, cls=SelfEncoder)
        else:
            result.status.code = 'FILE_NOT_FOUND'
            return json.dumps(result, cls=SelfEncoder)
        return json.dumps(result, cls=SelfEncoder)

    def unload_launch(self, request_json: LaunchFile) -> LaunchLoadReply:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.unload]")

        # Covert input dictionary into a proper python object
        request = request_json

        Log.debug(
            f"{self.__class__.__name__}: UnloadLaunch request:\n {request}")
        result = LaunchLoadReply(paths=[], changed_nodes=[], args=[])

        result.paths.append(request.path)
        # cfgid = CfgId(request.path, request.masteruri)
        # TODO: check if we need daemonuri as identification
        cfgid = CfgId(request.path, '')
        if cfgid in self._loaded_files:
            try:
                if cfgid in self._loaded_files:
                    self._remove_launch_from_observer(self._loaded_files[cfgid])
                    del self._loaded_files[cfgid]
                    result.status.code = 'OK'
                    # notify GUI about changes
                    self.websocket.publish('ros.launch.changed', {
                        'path': request.path, 'action': 'unloaded'})
                else:
                    result.status.code = 'ERROR'
                    result.status.msg = f"{request.path} not found"
            except Exception as e:
                err_text = "%s unloading failed!" % request.path
                err_details = "%s: %s" % (err_text, e)
                Log.warn("Unloading launch file: %s", err_details)
                result.status.code = 'ERROR'
                result.status.msg = err_details
        else:
            result.status.code = 'FILE_NOT_FOUND'
        return json.dumps(result, cls=SelfEncoder)

    def get_list(self) -> List[LaunchContent]:
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.get_list]")
        requested_files = list(self._loaded_files.keys())
        reply = []
        for cfgid in requested_files:
            lc = self._loaded_files[cfgid]
            reply_lc = LaunchContent(path=cfgid.path, args=[], masteruri=lc.daemonuri, host='',  # lc.host,
                                     nodes=[], parameters=[], associations=[])
            # Add launch arguments
            for name, p in lc.provided_launch_arguments:
                reply_lc.args.append(LaunchArgument(
                    name, p.value if hasattr(p, 'value') else p))

            nodes = lc.nodes()
            for item in nodes:
                reply_lc.nodes.append(item)
            reply.append(reply_lc)
        return json.dumps(reply, cls=SelfEncoder)

    def start_node(self, request_json: LaunchNode, return_as_json: bool = True) -> LaunchNodeReply:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.launch.start_node]")

        # Covert input dictionary into a proper python object
        request = request_json
        result = LaunchNodeReply(name=request.name, paths=[], launch_files=[])
        try:
            launch_configs = []
            if request.opt_launch:
                cfgid = CfgId(request.opt_launch, request.masteruri)
                if cfgid in self._loaded_files:
                    launch_configs.append(self._loaded_files[cfgid])
            if not launch_configs:
                # get launch configurations with given node
                launch_configs = []
                for cfgid, launchcfg in self._loaded_files.items():
                    n = launchcfg.get_node(request.name)
                    if n is not None:
                        Log.debug(f"Found launch file={launchcfg.filename};")
                        launch_configs.append(launchcfg)
            if not launch_configs:
                result.status.code = 'NODE_NOT_FOUND'
                result.status.msg = f"Node '{request.name}' not found"
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
            if len(launch_configs) > 1:
                result.status.code = 'MULTIPLE_LAUNCHES'
                result.status.msg = f"Node '{request.name}' found in multiple launch files"
                result.launch_files.extend(
                    [lcfg.filename for lcfg in launch_configs])
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
            try:
                result.launch_files.append(launch_configs[0].filename)
                executable_path = launch_configs[0].run_node(request.name)
                if executable_path:
                    if os.path.exists(executable_path):
                        if request.name not in self._node_exec:
                            self._node_exec[request.name] = executable_path
                            self._add_file_to_observe(executable_path)
                    else:
                        result.status.msg = executable_path
                Log.debug(f'Node={request.name}; start finished')
                result.status.code = 'OK'
            except exceptions.BinarySelectionRequest as bsr:
                result.status.code = 'MULTIPLE_BINARIES'
                result.status.msg = f"multiple binaries found for node '{request.name}': {bsr.choices}"
                result.paths.extend(bsr.choices)
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        except exceptions.ResourceNotFound as err_nf:
            result.status.code = 'ERROR'
            result.status.msg = f"Error while start node '{request.name}': {err_nf}"
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        except Exception as _errr:
            result.status.code = 'ERROR'
            result.status.msg = f"Error while start node '{request.name}': {traceback.format_exc()}"
            Log.warn(f"{self.__class__.__name__}: {result.status.msg}")
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        finally:
            nmd.launcher.server.screen_servicer.system_change()
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result

    def node_stopped(self, name: str) -> None:
        # remove executable path from observer
        if name in self._node_exec:
            self._remove_file_from_observe(self._node_exec[name])
            del self._node_exec[name]

    def start_nodes(self, request_json: List[LaunchNode], continue_on_error: bool = True) -> List[LaunchNodeReply]:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.launch.start_nodes]")

        result = []
        for request in request_json:
            node_result = self.start_node(request, return_as_json=False)
            result.append(node_result)
            if not continue_on_error:
                if result.status.code != 'OK':
                    break

        return json.dumps(result, cls=SelfEncoder)

    def get_included_files(self, request_json: LaunchIncludedFilesRequest) -> List[LaunchIncludedFile]:
        # Convert input dictionary into a proper python object
        request = request_json
        path = request.path
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.launch.get_included_files]: Path [{path}]")
        result = []
        cfg_included_files = []
        try:
            cfg = self._loaded_files[CfgId(request.path, '')]
            if cfg.launch_type == 'python':
                return cfg._included_files
            cfg_included_files.extend(cfg._included_files)
        except:
            pass
        # This part is executed if the launch file is not loaded or is of type XML
        # TODO add parser for python launch files
        try:
            search_in_ext = SEARCH_IN_EXT
            if request.search_in_ext:
                search_in_ext = request.search_in_ext
            # search for loaded file and get the arguments
            resolve_args = {arg.name: arg.value for arg in request.args if hasattr(arg, "value")}
            if not resolve_args:
                for cfgid, lcfg in self._loaded_files.items():
                    if cfgid.path == request.path:
                        resolve_args.update(lcfg.resolve_dict)
                        break
            # replay each file
            for inc_file in xml.find_included_files(request.path, recursive=request.recursive, unique=request.unique, search_in_ext=search_in_ext, resolve_args=resolve_args):
                file_size = 0
                if inc_file.exists:
                    file_size = os.path.getsize(inc_file.inc_path)
                args = [LaunchArgument(name=name, value=value) for name, value in inc_file.args.items()],
                default_inc_args = [LaunchArgument(name=name, value=value) for name, value in inc_file.args.items()],
                if cfg_included_files:
                    # use resolved launch arguments from loaded configuration
                    # remove if used: case if the same launch files was loaded multiple times with different arguments
                    if cfg_included_files[0].inc_path == os.path.realpath(inc_file.inc_path) and cfg_included_files[0].path == os.path.realpath(inc_file.path_or_str):
                        args = cfg_included_files[0].args
                        default_inc_args = cfg_included_files[0].args
                        del cfg_included_files[0]

                lincf = LaunchIncludedFile(path=inc_file.path_or_str,
                                           line_number=inc_file.line_number,
                                           inc_path=os.path.realpath(inc_file.inc_path),
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
        return json.dumps(result, cls=SelfEncoder)

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
                            if not arg_name in args:
                                request_args = True
                                break
                        if request_args:
                            req_args = []
                            for arg_name in args_in_name:
                                if arg_name in args:
                                    req_args.append(LaunchArgument(
                                        arg_name, args[arg_name]))
                                else:
                                    req_args.append(LaunchArgument(arg_name))
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
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.get_msg_struct]: msg [{msg_type}]")
        result = LaunchMessageStruct(msg_type)
        try:
            if self._is_action_type(msg_type):
                action_class, msg_class = self._get_action_types(msg_type)
            else:
                msg_class = get_message(msg_type)
            if not hasattr(msg_class, 'get_fields_and_field_types'):
                result.error_msg = f"unexpected message class: '{msg_class}', no 'get_fields_and_field_types' attribute found!"
                return json.dumps(result, cls=SelfEncoder)
            field_and_types = msg_class.get_fields_and_field_types()
            msg_dict = {'type': msg_type,
                        'name': '',
                        'def': self._expand_fields(field_and_types)}
            result.data = msg_dict
            result.valid = True
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            result.error_msg = repr(err)
            result.valid = False
        return json.dumps(result, cls=SelfEncoder)

    # create recursive dictionary for 'ros.launch.get_msg_struct'
    def _expand_fields(self, field_and_types):
        defs = []
        for field_name, field_type in field_and_types.items():
            field_struct = {'name': field_name, 'def': []}
            is_array = False
            base_type = field_type
            seq_length = None
            if field_type.startswith('sequence'):
                # handle sequences defined with sequence<>
                is_array = True
                type_re = re.search('<(\w[^,]*),?\s*(\S*)>', field_type)
                if type_re is not None:
                    base_type = type_re.group(1)
                    seq_length = type_re.group(2)
            elif '[' in field_type:
                # handle arrays defined with []
                is_array = True
                type_re = re.search(r'(.*)\[(\d*)\]', field_type)
                if type_re is not None:
                    base_type = type_re.group(1)
                    seq_length = type_re.group(2)
            if base_type not in [*rosidl_parser.definition.BASIC_TYPES, 'string', 'str', 'wstring']:
                # type is not a simple type
                # try to import message definition
                splitted_type = base_type.split('/')
                module = import_module('.msg', splitted_type[0])
                msg_class = getattr(module, splitted_type[1])
                if msg_class is not None:
                    sub_def = self._expand_fields(
                        msg_class.get_fields_and_field_types())
                field_struct['def'] = sub_def
            reported_type = base_type
            if (is_array):
                reported_type += f'[{seq_length}]' if seq_length else '[]'
            field_struct['type'] = reported_type
            field_struct['is_array'] = is_array
            if seq_length:
                field_struct['length'] = seq_length
            defs.append(field_struct)
        return defs

    def str2typedValue(self, value, value_type):
        result = value
        if 'int' in value_type:
            result = int(value)
        elif 'float' in value_type or 'double' in value_type:
            result = float(value)
        elif value_type.startswith('bool'):
            try:
                result = value.lower() in ("yes", "true", "t", "y", "1")
            except:
                pass
        return result

    def _str_from_dict(self, param_dict):
        result = dict()
        fields = param_dict if isinstance(
            param_dict, list) else param_dict['def']
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

    def publish_message(self, request_json: LaunchPublishMessage) -> None:
        try:
            # Convert input dictionary into a proper python object
            request = request_json
            Log.debug(
                f"{self.__class__.__name__}: Request to [ros.launch.publish_message]: msg [{request.msg_type}]")
            opt_str = ''
            opt_name_suf = '__latch_'
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
                opt_str += ' --use-rostime'
            fullname = f"/mas_publisher/{request.topic_name.strip('/')}".replace(
                '/', '_')
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
            screen_prefix = ' '.join([screen.get_cmd(fullname)])
            cmd = ' '.join([screen_prefix, 'ros2', 'topic', pub_cmd])
            Log.info(
                f"{self.__class__.__name__}: run ros2 publisher with: {cmd}")
            SupervisedPopen(shlex.split(cmd),
                            object_id=f"ros_topic_pub_{request.topic_name}", description=f"publish to topic {request.topic_name}")
        except Exception:
            import traceback
            print(traceback.format_exc())

    def _is_action_type(self, identifier: str) -> bool:
        return identifier.find("/action/") != -1

    def _get_action_types(self, identifier: str) -> Tuple[ActionClass, ActionRequestClass]:
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
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.get_srv_struct]: srv [{srv_type}]")
        result = LaunchMessageStruct(srv_type)
        try:
            if self._is_action_type(srv_type):
                action_class, request_class = self._get_action_types(srv_type)
            else:
                request_class = get_service(srv_type).Request
            if not hasattr(request_class, 'get_fields_and_field_types'):
                result.error_msg = f"unexpected service class: '{request_class}', no 'get_fields_and_field_types' attribute found!"
                return json.dumps(result, cls=SelfEncoder)
            field_and_types = request_class.get_fields_and_field_types()
            msg_dict = {'type': srv_type,
                        'name': '',
                        'def': self._expand_fields(field_and_types)}
            result.data = msg_dict
            result.valid = True
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            result.error_msg = repr(err)
            result.valid = False
        return json.dumps(result, cls=SelfEncoder)

    def call_service(self, request_json: LaunchCallService) -> None:
        # Convert input dictionary into a proper python object
        Log.info(f"{self.__class__.__name__}: Request to [ros.launch.call_service]: {request_json}")
        request = request_json
        result = LaunchMessageStruct(request.srv_type)
        try:
            # action looks like a service, but need a special handling
            if self._is_action_type(request.srv_type) or request.service_name.find("/_action/") != -1:
                action_class, request_class = self._get_action_types(request.srv_type)
                service_request = request_class()
                data = json.loads(request.data)
                response = nmd.launcher.call_action(request.service_name, action_class, service_request, 5)
            else:
                # call service
                request_class = get_service(request.srv_type)
                service_request = request_class.Request()
                data = json.loads(request.data)
                set_message_fields(service_request, self._str_from_dict(data))
                response = nmd.launcher.call_service(request.service_name, request_class, service_request, 5)
        except Exception as e:
            result.error_msg = 'Exception while calling service: %r' % e
        else:
            result.data = message_to_ordereddict(response)
            result.valid = True
        finally:
            nmd.ros_node.destroy_client(request.service_name)
        return json.dumps(result, cls=SelfEncoder)

    def get_message_types(self, mode: str = "message") -> str:
        Log.info(f"Request to [ros.launch.get_message_types]")
        result = []
        interfaces = {}
        if (mode == "service"):
            interfaces = get_service_interfaces()
        elif (mode == "action"):
            interfaces = get_action_interfaces()
        else:
            interfaces = get_message_interfaces()

        for pkg, messages in interfaces.items():
            for message in messages:
                result.append(f"{pkg}/{message}")
        return json.dumps(result, cls=SelfEncoder)

    def start_subscriber(self, request_json: SubscriberNode) -> bool:
        # Covert input dictionary into a proper python object
        request = request_json
        topic = request.topic
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.subscriber.start]: {topic}")
        package_name = 'fkie_mas_daemon'
        executable = 'mas-subscriber'
        cmd = f"ros2 run {package_name} {executable}"
        # fullname = f"/_mas_subscriber/{topic.replace('.', '/').strip('/')}"
        self.namespace, self.name = ros2_subscriber_nodename_tuple(topic)
        fullname = os.path.join(self.namespace, self.name)
        # args = [f"__name:={fullname}"]
        # args.append(f'--name={fullname}')
        args = []
        args.append(f'--ws_port={self.ws_port}')
        args.append(f'--topic={topic}')
        args.append(f'--message_type={request.message_type}')
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
        if hasattr(request, "qos") and request.qos:
            if request.qos.durability:
                args.append(f'--qos-durability={RosQos.durabilityToString(request.qos.durability)}')
            if request.qos.reliability:
                args.append(f'--qos-reliability={RosQos.reliabilityToString(request.qos.reliability)}')
            if request.qos.liveliness:
                args.append(f'--qos-liveliness={RosQos.livelinessToString(request.qos.liveliness)}')
        else:
            # TODO wait for publisher and detect qos
            pass

        # run on local host
        screen_prefix = ' '.join([screen.get_cmd(fullname)])
        # set environment
        new_env = dict(os.environ)

        # set display variable to local display
        if 'DISPLAY' in new_env:
            if not new_env['DISPLAY'] or new_env['DISPLAY'] == 'remote':
                del new_env['DISPLAY']
        else:
            new_env['DISPLAY'] = ':0'

        # start
        Log.info(
            f"{self.__class__.__name__}: {screen_prefix} {cmd} {' '.join(args)}")
        # Log.debug(
        #     f"environment while run node '{fullname}': '{new_env}'")
        # Log.debug(
        #     f"args while run node '{fullname}': '{args}', JOINED: {' '.join([screen_prefix, cmd] + args)}")
        SupervisedPopen(shlex.split(' '.join([screen_prefix, cmd] + args)), env=new_env,
                        object_id=f"run_node_{fullname}", description=f"Run [{package_name}]{executable}")
        return True

    def list_nodes(self):
        result = []
        for cfgid in list(self._loaded_files.keys()):
            lc = self._loaded_files[cfgid]
            for item in lc.roscfg.nodes:
                node_fullname = ns_join(item.namespace, item.name)
                result.append(node_fullname)
        return result
