# The MIT License (MIT)

# Copyright (c) 2014-2024 Fraunhofer FKIE, Alexander Tiderko

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from typing import Text
import os
import sys
import threading
import time
import websockets
import rclpy

# crossbar-io dependencies
import asyncio
import json
from types import SimpleNamespace

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
from fkie_mas_pylib.launch import xml
from fkie_mas_pylib.names import ns_join
from fkie_mas_pylib.crossbar import server
from fkie_mas_pylib.settings import Settings
from fkie_mas_pylib.system.host import get_host_name
from fkie_mas_pylib.system.timer import RepeatTimer
from fkie_mas_pylib.crossbar.base_session import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket import WebSocketHandler
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
    def __init__(self, rosnode, *, default_domain_id=-1):
        self.rosnode = rosnode
        self.crossbar_port = server.port()
        self.crossbar_realm = "ros"
        self.crossbar_loop = asyncio.get_event_loop()
        self._version, self._date = detect_version(
            nmd.ros_node, "fkie_mas_daemon"
        )
        self._settings = Settings(version=self._version)
        self.ros_domain_id = default_domain_id
        if self.ros_domain_id > 0:
            rosnode.get_logger().warn(
                "default ROS_DOMAIN_ID=%d overwritten to %d" % (
                    0, self.ros_domain_id)
            )
        self.name = get_host_name()
        self.uri = ""
        self._timer_crossbar_ready = None
        self.monitor_servicer = MonitorServicer(
            self._settings, self.crossbar_loop, self.crossbar_realm, self.crossbar_port
        )
        self.file_servicer = FileServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port
        )
        self.screen_servicer = ScreenServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port
        )
        self.rosstate_servicer = RosStateServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port
        )
        self.parameter_servicer = ParameterServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port
        )
        self.launch_servicer = LaunchServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            ros_domain_id=self.ros_domain_id,
        )
        self.version_servicer = VersionServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port
        )

        rosnode.create_service(
            LoadLaunch, "~/start_launch", self._rosservice_start_launch
        )
        rosnode.create_service(
            LoadLaunch, "~/load_launch", self._rosservice_load_launch
        )
        rosnode.create_service(Task, "~/run", self._rosservice_start_node)
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            # history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub_endpoint = rosnode.create_publisher(
            Endpoint, "daemons", qos_profile=qos_profile
        )
        self.rosname = ns_join(
            nmd.ros_node.get_namespace(), nmd.ros_node.get_name())
        self._endpoint_msg = Endpoint(
            name=self.name,
            uri=self.rosstate_servicer.uri,
            ros_name=self.rosname,
            ros_domain_id=self.ros_domain_id,
            on_shutdown=False,
            pid=os.getpid(),
        )

    def __del__(self):
        self.crossbar_loop.stop()
        self.version_servicer = None
        self.launch_servicer = None
        self.monitor_servicer = None
        self.screen_servicer = None
        self.rosstate_servicer = None
        self.parameter_servicer = None

    async def ws_handler(self, websocket, path):
        handler = WebSocketHandler(websocket, path, self.crossbar_loop)
        await handler.spin()
        # self.ws_connections.add(websocket)
        # print(f"path: {path}")
        # try:
        #     async for message in websocket:
        #         print(message)
        #         await websocket.send("Hi")
        # except websockets.ConnectionClosedError:
        #     pass
        # finally:
        #     self.ws_connections.remove(websocket)

    async def websocket_main(self, port=35430):
        nmd.ros_node.get_logger().info(
            f"Open Websocket on port {port}")
        try:
            async with websockets.serve(self.ws_handler, "0.0.0.0", port):
                while rclpy.ok():
                    await asyncio.sleep(1)
                # try:
                #     await asyncio.Future()
                # except:
                #     import traceback
                #     print(traceback.format_exc())
        except:
            import traceback
            print(traceback.format_exc())

    def start(self, port: int, displayed_name: Text = "") -> bool:
        if displayed_name:
            self.name = displayed_name
        port = self.crossbar_port
        # update name if port is not a default one
        self.insecure_port = port
        if server.port() != port:
            self.name = f"{self.name}_{port}"

        asyncio.run_coroutine_threadsafe(
            self.websocket_main(), self.crossbar_loop)
        nmd.ros_node.get_logger().info(
            f"Connect to crossbar server on port {port}")
        self._endpoint_msg.name = self.name
        self._crossbarThread = threading.Thread(
            target=self.run_crossbar_forever, args=(
                self.crossbar_loop,), daemon=True
        )
        self._crossbarThread.start()
        self._crossbarNotificationThread = threading.Thread(
            target=self._crossbar_notify_if_regsitered, daemon=True
        )
        self._crossbarNotificationThread.start()
        self.rosstate_servicer.start()
        self.screen_servicer.start()
        return True

    async def crossbar_connect(self) -> None:
        current_task = asyncio.current_task()
        if not self.crossbar_connecting:
            task = asyncio.create_task(self.crossbar_connect_async())
        else:
            task = current_task
        await asyncio.gather(task)

    def run_crossbar_forever(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def _crossbar_notify_if_regsitered(self):
        registration_finished = False
        while not registration_finished:
            registration_finished = True
            registration_finished &= self.launch_servicer.crossbar_registered
            registration_finished &= self.screen_servicer.crossbar_registered
            registration_finished &= self.rosstate_servicer.crossbar_registered
            registration_finished &= self.parameter_servicer.crossbar_registered
            registration_finished &= self.version_servicer.crossbar_registered
            registration_finished &= self.monitor_servicer.crossbar_registered
            time.sleep(0.5)
        self.publish_daemon_state(True)
        self._crossbar_send_status(True)

    def _crossbar_send_status(self, status: bool):
        # try to send notification to crossbar subscribers
        self.rosstate_servicer.publish_to(
            "ros.daemon.ready", {"status": status, 'timestamp': time.time() * 1000})
        if status:
            if self._timer_crossbar_ready is None:
                self._timer_crossbar_ready = RepeatTimer(3.0,
                                                         self._crossbar_send_status, args=(
                                                             True,))
                self._timer_crossbar_ready.start()
        else:
            if self._timer_crossbar_ready is not None:
                self._timer_crossbar_ready.cancel()
                self._timer_crossbar_ready = None

    def publish_daemon_state(self, is_running: bool = True):
        self._endpoint_msg.on_shutdown = not is_running
        try:
            self.pub_endpoint.publish(self._endpoint_msg)
        except Exception as a:
            pass
            return
            import traceback

            print(traceback.format_exc())

    def shutdown(self):
        WAIT_TIMEOUT = 3
        self.publish_daemon_state(False)
        self._crossbar_send_status(False)
        self.version_servicer.stop()
        self.screen_servicer.stop()
        self.launch_servicer.stop()
        self.file_servicer.stop()
        self.monitor_servicer.stop()
        self.rosstate_servicer.stop()
        self.screen_servicer.stop()
        self.parameter_servicer.stop()
        shutdown_task = self.crossbar_loop.create_task(
            self.crossbar_loop.shutdown_asyncgens()
        )
        self.rosnode.destroy_publisher(self.pub_endpoint)
        sleep_time = 0.5
        while not shutdown_task.done() or self.parameter_servicer.crossbar_connected:
            time.sleep(sleep_time)
            sleep_time += 0.5
            if sleep_time > WAIT_TIMEOUT:
                break

    def load_launch_file(self, path, autostart=False):
        pass
        # self.launch_servicer.load_launch_file(xml.interpret_path(path), autostart)

    def _rosservice_start_launch(self, request, response):
        nmd.ros_node.get_logger().info(
            "Service request to load and start %s" % request.path
        )
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
        result = self.launch_servicer.load_launch(params, return_as_json=False)
        if result.status.code != "OK":
            raise Exception(result.status.msg)
        # self.launch_servicer.load_launch_file(xml.interpret_path(request.path), True)
        return response

    def _rosservice_load_launch(self, request, response):
        nmd.ros_node.get_logger().info("Service request to load %s" % request.path)
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
        result = self.launch_servicer.load_launch(params, return_as_json=False)
        if result.status.code != "OK":
            raise Exception(result.status.msg)
        return response

    def _rosservice_start_node(self, request, response):
        """
        Callback for the ROS service to start a node.
        """
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
        result = self.launch_servicer.start_node(params, return_as_json=False)
        if result.status.code != "OK":
            raise Exception(result.status.msg)
        return response
