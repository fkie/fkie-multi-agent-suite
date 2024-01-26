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
from autobahn import wamp
import json

from fkie_mas_daemon.monitor.service import Service

from fkie_mas_pylib.crossbar.runtime_interface import DiagnosticArray
from fkie_mas_pylib.crossbar.runtime_interface import DiagnosticStatus
from fkie_mas_pylib.crossbar.runtime_interface import SystemEnvironment
from fkie_mas_pylib.crossbar.runtime_interface import SystemInformation
from fkie_mas_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_mas_pylib.crossbar.base_session import SelfEncoder
from fkie_mas_pylib.logging.logging import Log


class MonitorServicer(CrossbarBaseSession):
    def __init__(
        self, settings, loop: asyncio.AbstractEventLoop, realm: str = "ros", port: int = 11911
    ):
        Log.info("Create monitor servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port)
        self._monitor = Service(settings, self.diagnosticsCbPublisher)

    def stop(self):
        self._monitor.stop()
        self.shutdown()

    @wamp.register("ros.provider.get_system_info")
    def getSystemInfo(self) -> SystemInformation:
        Log.info("crossbar: get system info")
        return json.dumps(SystemInformation(), cls=SelfEncoder)

    @wamp.register("ros.provider.get_system_env")
    def getSystemEnv(self) -> SystemEnvironment:
        Log.info("crossbar: get system env")
        return json.dumps(SystemEnvironment(), cls=SelfEncoder)

    def _toCrossbarDiagnostics(self, rosmsg):
        cbMsg = DiagnosticArray(
            timestamp=float(rosmsg.header.stamp.sec)
            + float(rosmsg.header.stamp.nanosec) / 1000000000.0, status=[]
        )
        for sensor in rosmsg.status:
            values = []
            for v in sensor.values:
                values.append(DiagnosticStatus.KeyValue(v.key, v.value))
            status = DiagnosticStatus(
                sensor.level, sensor.name, sensor.message, sensor.hardware_id, values
            )
            cbMsg.status.append(status)
        return cbMsg

    def diagnosticsCbPublisher(self, rosmsg):
        self.publish_to(
            "ros.provider.diagnostics",
            json.dumps(self._toCrossbarDiagnostics(rosmsg), cls=SelfEncoder),
        )

    @wamp.register("ros.provider.get_diagnostics")
    def getDiagnostics(self) -> DiagnosticArray:
        Log.info("crossbar: get diagnostics")
        rosmsg = self._monitor.get_diagnostics(0, 0)
        # copy message to the crossbar structure
        return json.dumps(self._toCrossbarDiagnostics(rosmsg), cls=SelfEncoder)

    @wamp.register("ros.provider.ros_clean_purge")
    def rosCleanPurge(self) -> {bool, str}:
        Log.info("crossbar: ros_clean_purge")
        result = False
        message = 'Not implemented'
        Log.warn("Not implemented: ros.provider.ros_clean_purge")
        return json.dumps({result: result, message: message}, cls=SelfEncoder)
