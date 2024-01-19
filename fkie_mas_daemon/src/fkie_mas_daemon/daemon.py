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
from fkie_mas_pylib.crossbar import server
from fkie_mas_pylib.crossbar.base_session import SelfEncoder
from fkie_mas_pylib.settings import Settings
from fkie_mas_daemon.version import detect_version


# crossbar-io dependencies
import asyncio

from .file_servicer import FileServicer
from .launch_servicer import LaunchServicer
from .monitor_servicer import MonitorServicer
from .screen_servicer import ScreenServicer
from .version_servicer import VersionServicer
from .parameter_servicer import ParameterServicer


class MASDaemon:
    def __init__(self, test_env=False):
        self.crossbar_port = server.port()
        self.crossbar_realm = "ros"
        self.crossbar_loop = asyncio.get_event_loop()
        self.server = None
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
        self.crossbar_loop.stop()
        self.server.stop(3)

    def _update_parameter(self, settings):
        # self._verbosity = settings.param("global/verbosity", "INFO")
        pass

    def start(self, port=None):
        crossbar_port = port if (port != None) else self.crossbar_port
        Log.info(f"Start crossbar with port {crossbar_port}")
        self.monitor_servicer = MonitorServicer(
            self._settings,
            self.crossbar_loop,
            self.crossbar_realm,
            crossbar_port,
            test_env=self._test_env,
        )
        self.launch_servicer = LaunchServicer(
            self.monitor_servicer,
            self.crossbar_loop,
            self.crossbar_realm,
            crossbar_port,
            test_env=self._test_env,
        )
        self.parameter_servicer = ParameterServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            crossbar_port,
            test_env=self._test_env,
        )
        self.file_servicer = FileServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            crossbar_port,
            test_env=self._test_env,
        )
        self.screen_servicer = ScreenServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            crossbar_port,
            test_env=self._test_env,
        )
        self.version_servicer = VersionServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            crossbar_port,
            test_env=self._test_env,
        )

        # Log.info(f"Connect to crossbar server @ ws://localhost:{self.crossbar_port}/ws, realm: {self.crossbar_realm}")
        self._crossbarThread = threading.Thread(
            target=self.run_crossbar_forever, args=(
                self.crossbar_loop,), daemon=True
        )
        self._crossbarThread.start()
        self._crossbarNotificationThread = threading.Thread(
            target=self._crossbar_notify_if_regsitered, daemon=True
        )
        self._crossbarNotificationThread.start()

    def run_crossbar_forever(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def _crossbar_notify_if_regsitered(self):
        registration_finished = False
        while not registration_finished:
            registration_finished = True
            registration_finished &= self.launch_servicer.crossbar_registered
            registration_finished &= self.screen_servicer.crossbar_registered
            registration_finished &= self.file_servicer.crossbar_registered
            registration_finished &= self.parameter_servicer.crossbar_registered
            registration_finished &= self.version_servicer.crossbar_registered
            time.sleep(0.5)
        self._crossbar_send_status(True)

    def _crossbar_send_status(self, status: bool):
        # try to send notification to crossbar subscribers
        self.launch_servicer.publish_to("ros.daemon.ready", {"status": status})

    def shutdown(self):
        WAIT_TIMEOUT = 3
        self._crossbar_send_status(False)
        shutdown_task = self.crossbar_loop.create_task(
            self.crossbar_loop.shutdown_asyncgens()
        )
        self.version_servicer.stop()
        self.launch_servicer.stop()
        self.monitor_servicer.stop()
        self.parameter_servicer.shutdown()
        self.file_servicer.shutdown()
        self.screen_servicer.stop()
        self.server.stop(WAIT_TIMEOUT)
        sleep_time = 0.5
        while not shutdown_task.done() or self.screen_servicer.crossbar_connected:
            time.sleep(sleep_time)
            sleep_time += 0.5
            if sleep_time > WAIT_TIMEOUT:
                print("break")
                break

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
