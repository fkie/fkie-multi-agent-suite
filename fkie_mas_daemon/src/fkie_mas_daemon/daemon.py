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

from concurrent import futures
import json
import rospy
import os
import threading
import time

from fkie_mas_msgs.srv import ListNodes
from fkie_mas_msgs.srv import ListNodesResponse
from fkie_mas_msgs.srv import LoadLaunch
from fkie_mas_msgs.srv import LoadLaunchResponse
from fkie_mas_msgs.srv import Task
from fkie_mas_pylib.launch import xml
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.settings import Settings
from fkie_mas_pylib.system.timer import RepeatTimer
from fkie_mas_pylib.websocket import ws_port
from fkie_mas_pylib.websocket.server import WebSocketServer
from fkie_mas_daemon.version import detect_version


import asyncio
import websockets

from .file_servicer import FileServicer
from .launch_servicer import LaunchServicer
from .monitor_servicer import MonitorServicer
from .screen_servicer import ScreenServicer
from .version_servicer import VersionServicer
from .parameter_servicer import ParameterServicer


class MASDaemon:
    def __init__(self, test_env=False):
        self.ws_port = ws_port()
        self.asyncio_loop = asyncio.get_event_loop()
        self._timer_daemon_ready = None
        self._test_env = test_env
        self._version, self._date = detect_version("fkie_mas_daemon")
        self._settings = Settings(version=self._version)
        self._settings.add_reload_listener(self._update_parameter)
        self.version_servicer = None
        self.launch_servicer = None
        self.monitor_servicer = None
        self.parameter_servicer = None
        self.file_servicer = None
        self.screen_servicer = None
        self.ws_server = WebSocketServer(self.asyncio_loop)
        rospy.Service("~start_launch", LoadLaunch,
                      self._rosservice_start_launch)
        rospy.Service("~load_launch", LoadLaunch, self._rosservice_load_launch)
        rospy.Service("~run", Task, self._rosservice_start_node)
        rospy.Service("~list_nodes", ListNodes, self._rosservice_list_nodes)

    def __del__(self):
        self.version_servicer = None
        self.launch_servicer = None
        self.monitor_servicer = None
        self.parameter_servicer = None
        self.file_servicer = None
        self.screen_servicer = None
        self.asyncio_loop.stop()

    def _update_parameter(self, settings):
        # self._verbosity = settings.param("global/verbosity", "INFO")
        pass

    def start(self, port=None):
        use_port = port if (port != None) else self.ws_port
        self.ws_server.start(port)
        self.monitor_servicer = MonitorServicer(
            self._settings,
            test_env=self._test_env,
        )
        self.launch_servicer = LaunchServicer(
            self.monitor_servicer,
            test_env=self._test_env,
        )
        self.parameter_servicer = ParameterServicer(
            test_env=self._test_env,
        )
        self.file_servicer = FileServicer(
            test_env=self._test_env,
        )
        self.screen_servicer = ScreenServicer(
            test_env=self._test_env,
        )
        self.version_servicer = VersionServicer(
            test_env=self._test_env,
        )

        Log.info(f"Start websocket server @ ws://0.0.0.0:{use_port}")
        self._wsThread = threading.Thread(
            target=self.asyncio_loop.run_forever, daemon=True
        )
        self._wsThread.start()
        self._daemon_send_status(True)

    def _daemon_send_status(self, status: bool):
        # try to send notification to websocket subscribers
        self.ws_server.publish("ros.daemon.ready", {
            "status": status, 'timestamp': time.time() * 1000})
        if status:
            if self._timer_daemon_ready is None:
                self._timer_daemon_ready = RepeatTimer(3.0,
                                                       self._daemon_send_status, args=(
                                                           True,))
                self._timer_daemon_ready.start()
        else:
            if self._timer_daemon_ready is not None:
                self._timer_daemon_ready.cancel()
                self._timer_daemon_ready = None

    def shutdown(self):
        self._daemon_send_status(False)
        self.version_servicer.stop()
        self.launch_servicer.stop()
        self.monitor_servicer.stop()
        self.parameter_servicer.stop()
        self.file_servicer.stop()
        self.screen_servicer.stop()

    def load_launch_file(self, path, autostart=False):
        self.launch_servicer.load_launch_file(
            xml.interpret_path(path), autostart)

    def _rosservice_start_launch(self, request):
        Log.info("Service request to load and start %s" % request.path)
        self.launch_servicer.load_launch_file(
            xml.interpret_path(request.path), True)
        return LoadLaunchResponse()

    def _rosservice_load_launch(self, request):
        Log.info("Service request to load %s" % request.path)
        self.launch_servicer.load_launch_file(
            xml.interpret_path(request.path), False)
        return LoadLaunchResponse()

    def _rosservice_start_node(self, req):
        """
        Callback for the ROS service to start a node.
        """
        self.launch_servicer.start_node_by_name(req.node)
        return []

    def _rosservice_list_nodes(self, req):
        """
        Callback for the ROS service to list all nodes loaded by all launch files.
        """
        return ListNodesResponse(self.launch_servicer.list_nodes())
