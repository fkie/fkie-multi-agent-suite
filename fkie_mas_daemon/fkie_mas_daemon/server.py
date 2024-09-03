# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from typing import Text
import os
import time
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from fkie_mas_msgs.msg import Endpoint
from fkie_mas_msgs.srv import ListNodes
from fkie_mas_msgs.srv import LoadLaunch
from fkie_mas_msgs.srv import Task
from fkie_mas_pylib.interface.launch_interface import LaunchNode
from fkie_mas_pylib.interface.launch_interface import LaunchLoadRequest
from fkie_mas_pylib.launch import xml
from fkie_mas_pylib.names import ns_join
from fkie_mas_pylib.settings import Settings
from fkie_mas_pylib.system.host import get_host_name
from fkie_mas_pylib.system.timer import RepeatTimer
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket import ws_port
from fkie_mas_pylib.websocket.server import WebSocketServer
import fkie_mas_daemon as nmd

from fkie_mas_daemon.file_servicer import FileServicer
from fkie_mas_daemon.launch_servicer import LaunchServicer
from fkie_mas_daemon.monitor_servicer import MonitorServicer
from fkie_mas_daemon.parameter_servicer import ParameterServicer
from fkie_mas_daemon.rosstate_servicer import RosStateServicer
from fkie_mas_daemon.screen_servicer import ScreenServicer
from fkie_mas_daemon.version_servicer import VersionServicer
from fkie_mas_daemon.version import detect_version


class Server:
    def __init__(self, ros_node, *, default_domain_id=-1):
        self.ros_node = ros_node
        self.ws_port = ws_port()
        self._version, self._date = detect_version(
            nmd.ros_node, "fkie_mas_daemon"
        )
        self.ros_node.create_service(
            LoadLaunch, "~/start_launch", self._rosservice_start_launch
        )
        self.ros_node.create_service(
            LoadLaunch, "~/load_launch", self._rosservice_load_launch
        )
        self.ros_node.create_service(
            Task, "~/run", self._rosservice_start_node)
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            # history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub_endpoint = self.ros_node.create_publisher(
            Endpoint, "daemons", qos_profile=qos_profile
        )
        self.ws_server = WebSocketServer()
        self._settings = Settings(version=self._version)
        self.ros_domain_id = default_domain_id
        if self.ros_domain_id > 0:
            ros_node.get_logger().warn(
                "default ROS_DOMAIN_ID=%d overwritten to %d" % (
                    0, self.ros_domain_id)
            )
        self.name = get_host_name()
        self.uri = f"ws://{get_host_name()}:{self.ws_port}"
        self._timer_ws_ready = None
        self.monitor_servicer = MonitorServicer(
            self._settings, self.ws_server)
        self.file_servicer = FileServicer(self.ws_server)
        self.screen_servicer = ScreenServicer(self.ws_server)
        self.rosstate_servicer = RosStateServicer(self.ws_server)
        self.parameter_servicer = ParameterServicer(self.ws_server)
        self.launch_servicer = LaunchServicer(
            self.ws_server,
            ws_port=self.ws_port,
        )
        self.version_servicer = VersionServicer(self.ws_server, self.rosstate_servicer)

        self.rosname = ns_join(
            nmd.ros_node.get_namespace(), nmd.ros_node.get_name())
        self._endpoint_msg = Endpoint(
            name=self.name,
            uri=self.uri,
            ros_name=self.rosname,
            ros_domain_id=self.ros_domain_id,
            on_shutdown=False,
            pid=os.getpid(),
        )
        self._ws_thread = None

    def __del__(self):
        self.version_servicer = None
        self.launch_servicer = None
        self.monitor_servicer = None
        self.screen_servicer = None
        self.rosstate_servicer = None
        self.parameter_servicer = None

    def start(self, port: int, displayed_name: Text = "") -> bool:
        Log.info(f"start websocket server on port {port}")
        if displayed_name:
            self.name = displayed_name
        # update name if port is not a default one
        self.insecure_port = port
        if ws_port() != port:
            self.name = f"{self.name}_{port}"
        self._endpoint_msg.name = self.name
        self._endpoint_msg.uri = f"ws://{get_host_name()}:{port}"
        self.ws_server.start_threaded(port)
        self.rosstate_servicer.start()
        self.screen_servicer.start()
        self.publish_daemon_state(True)
        self._ws_send_status(True)
        return True

    def _ws_send_status(self, status: bool):
        # try to send notification to websocket subscribers
        self.ws_server.publish(
            "ros.daemon.ready", {"status": status, 'timestamp': time.time() * 1000})
        if status:
            if self._timer_ws_ready is None:
                self._timer_ws_ready = RepeatTimer(3.0,
                                                   self._ws_send_status, args=(
                                                       True,))
                self._timer_ws_ready.start()
        else:
            if self._timer_ws_ready is not None:
                self._timer_ws_ready.cancel()
                self._timer_ws_ready = None

    def publish_daemon_state(self, is_running: bool = True):
        self._endpoint_msg.on_shutdown = not is_running
        try:
            self.pub_endpoint.publish(self._endpoint_msg)
        except Exception as a:
            pass
            # import traceback
            # print(traceback.format_exc())

    def shutdown(self):
        self.publish_daemon_state(False)
        self._ws_send_status(False)
        self.version_servicer.stop()
        self.screen_servicer.stop()
        self.launch_servicer.stop()
        self.file_servicer.stop()
        self.monitor_servicer.stop()
        self.rosstate_servicer.stop()
        self.screen_servicer.stop()
        self.parameter_servicer.stop()
        self.ros_node.destroy_publisher(self.pub_endpoint)

    def load_launch_file(self, path, autostart=False):
        pass
        # self.launch_servicer.load_launch_file(xml.interpret_path(path), autostart)

    async def _rosservice_start_launch(self, request, response):
        Log.info(f"call service to load and start {request.path}")
        params = {
            "ros_package": "",
            "launch": "",
            "path": xml.interpret_path(request.path),
            "args": [],
            "force_first_file": False,
            "request_args": [],
            "masteruri": "",
            "host": "",
        }
        result = self.launch_servicer.load_launch(LaunchLoadRequest(path=request.path), return_as_json=False)
        if result.status.code != "OK":
            raise Exception(result.status.msg)
        # TODO start nodes
        return response

    async def _rosservice_load_launch(self, request, response):
        Log.info(f"call service to load {request.path}")
        params = {
            "ros_package": "",
            "launch": "",
            "path": xml.interpret_path(request.path),
            "args": [],
            "force_first_file": False,
            "request_args": [],
            "masteruri": "",
            "host": "",
        }

        result = self.launch_servicer.load_launch(
            LaunchLoadRequest(path=request.path), return_as_json=False)
        if result.status.code != "OK":
            raise Exception(result.status.msg)
        return response

    async def _rosservice_start_node(self, request, response):
        """
        Callback for the ROS service to start a node.
        """
        Log.info(f"call service to start node {request.node}")
        params = {
            "name": request.node,
            "opt_binary": "",
            "opt_launch": "",
            "loglevel": "",
            "logformat": "",
            "masteruri": "",
            "reload_global_param": False,
            "cmd_prefix": "",
        }
        result = self.launch_servicer.start_node(LaunchNode(name=request.node), return_as_json=False)
        if result.status.code != "OK":
            raise Exception(result.status.msg)
        return response
