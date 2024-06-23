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


import asyncio
import json
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.runtime_interface import DaemonVersion
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket.server import WebSocketServer
import fkie_mas_daemon as nmd
from . import version


class VersionServicer:
    def __init__(self, websocket: WebSocketServer):
        Log.info("Create ROS2 version servicer")
        self._version, self._date = version.detect_version(
            nmd.ros_node, "fkie_mas_daemon"
        )
        websocket.register("ros.daemon.get_version", self.get_version)

    def stop(self):
        pass

    def get_version(self) -> DaemonVersion:
        Log.info(f"{self.__class__.__name__}: get daemon version ")
        reply = DaemonVersion(self._version, self._date)
        return json.dumps(reply, cls=SelfEncoder)
