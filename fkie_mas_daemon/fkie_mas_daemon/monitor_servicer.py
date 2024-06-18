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
import os
import psutil
import signal
import threading


from fkie_mas_daemon.monitor.service import Service

from fkie_mas_pylib.interface.runtime_interface import DiagnosticArray
from fkie_mas_pylib.interface.runtime_interface import DiagnosticStatus
from fkie_mas_pylib.interface.runtime_interface import SystemEnvironment
from fkie_mas_pylib.interface.runtime_interface import SystemInformation
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.defines import SETTINGS_PATH
from fkie_mas_pylib.websocket import ws_publish_to, ws_register_method


class MonitorServicer:
    def __init__(self, settings, loop: asyncio.AbstractEventLoop):
        Log.info("Create monitor servicer")
        self._monitor = Service(settings, self.diagnosticsCbPublisher)
        ws_register_method("ros.provider.get_system_info", self.getSystemInfo)
        ws_register_method("ros.provider.get_system_env", self.getSystemEnv)
        ws_register_method("ros.provider.get_warnings",
                           self.getProviderWarnings)
        ws_register_method("ros.provider.get_diagnostics", self.getDiagnostics)
        ws_register_method("ros.provider.ros_clean_purge", self.rosCleanPurge)
        ws_register_method("ros.provider.shutdown", self.rosShutdown)

    def stop(self):
        self._monitor.stop()
        self.shutdown()

    def getSystemInfo(self) -> SystemInformation:
        Log.info("request: get system info")
        return json.dumps(SystemInformation(), cls=SelfEncoder)

    def getSystemEnv(self) -> SystemEnvironment:
        Log.info("request: get system env")
        return json.dumps(SystemEnvironment(), cls=SelfEncoder)

    def getProviderWarnings(self) -> str:
        Log.info('Request to [ros.provider.get_warnings]')
        # TODO collect warnings
        return json.dumps([], cls=SelfEncoder)

    def _toJsonDiagnostics(self, rosmsg):
        cbMsg = DiagnosticArray(
            timestamp=float(rosmsg.header.stamp.sec)
            + float(rosmsg.header.stamp.nanosec) / 1000000000.0, status=[]
        )
        for sensor in rosmsg.status:
            values = []
            for v in sensor.values:
                values.append(DiagnosticStatus.KeyValue(v.key, v.value))
            level = int.from_bytes(sensor.level, byteorder='big')
            status = DiagnosticStatus(
                level, sensor.name, sensor.message, sensor.hardware_id, values
            )
            cbMsg.status.append(status)
        return cbMsg

    def diagnosticsCbPublisher(self, rosmsg):
        ws_publish_to("ros.provider.diagnostics",
                      json.dumps(self._toJsonDiagnostics(rosmsg), cls=SelfEncoder),)

    def getDiagnostics(self) -> DiagnosticArray:
        Log.info("request: get diagnostics")
        rosmsg = self._monitor.get_diagnostics(0, 0)
        # copy message to the JSON structure
        return json.dumps(self._toJsonDiagnostics(rosmsg), cls=SelfEncoder)

    def rosCleanPurge(self) -> {bool, str}:
        Log.info("request: ros_clean_purge")
        result = False
        message = 'Not implemented'
        Log.warn("Not implemented: ros.provider.ros_clean_purge")
        return json.dumps({result: result, message: message}, cls=SelfEncoder)

    def rosShutdown(self) -> {bool, str}:
        Log.info("ros.provider.shutdown")
        result = False
        message = ''
        procs = []
        try:
            for process in psutil.process_iter():
                cmdStr = ' '.join(process.cmdline())
                if cmdStr.find(SETTINGS_PATH) > -1:
                    if (cmdStr.find('mas-daemon') == -1):
                        # ignore mas daemon pid to kill it last
                        procs.append(process)
                        process.terminate()
            gone, alive = psutil.wait_procs(procs, timeout=3)
            for p in alive:
                p.kill()
            result = True
        except Exception as error:
            import traceback
            print(traceback.format_exc())
            message = str(error)
        screen.wipe()
        return json.dumps({result: result, message: message}, cls=SelfEncoder)

    def _killSelf(self, pidList=[], sig=signal.SIGTERM):
        for pid in pidList:
            os.kill(pid, sig)
        os.kill(os.getpid(), signal.SIGINT)
