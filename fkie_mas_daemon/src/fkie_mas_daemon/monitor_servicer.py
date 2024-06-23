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

from typing import List
import json
import os
import psutil
import signal
import threading
from types import SimpleNamespace
import sys
import rospy
from fkie_mas_pylib.websocket.server import WebSocketServer

try:
    from roscpp.srv import GetLoggers, SetLoggerLevel, SetLoggerLevelRequest
except ImportError as err:
    sys.stderr.write("Cannot import GetLoggers service definition: %s" % err)


from fkie_mas_daemon.monitor.service import Service
from fkie_mas_pylib.interface.runtime_interface import DiagnosticArray
from fkie_mas_pylib.interface.runtime_interface import DiagnosticStatus
from fkie_mas_pylib.interface.runtime_interface import LoggerConfig
from fkie_mas_pylib.interface.runtime_interface import SystemEnvironment
from fkie_mas_pylib.interface.runtime_interface import SystemInformation
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.defines import SETTINGS_PATH


class MonitorServicer:
    def __init__(self, settings, websocket: WebSocketServer, test_env=False):
        Log.info("Create monitor servicer")
        self._monitor = Service(settings, self.diagnosticsCbPublisher)
        self.websocket = websocket
        websocket.register("ros.provider.get_system_info", self.getSystemInfo)
        websocket.register("ros.provider.get_system_env", self.getSystemEnv)
        websocket.register("ros.provider.get_diagnostics", self.getDiagnostics)
        websocket.register("ros.provider.ros_clean_purge", self.rosCleanPurge)
        websocket.register("ros.provider.shutdown", self.rosShutdown)
        websocket.register("ros.nodes.get_loggers", self.getLoggers)
        websocket.register("ros.nodes.set_logger_level", self.setLoggerLevel)

    def stop(self):
        self._monitor.stop()

    def _toJsonDiagnostics(self, rosmsg):
        cbMsg = DiagnosticArray(
            float(rosmsg.header.stamp.secs)
            + float(rosmsg.header.stamp.nsecs) / 1000000000.0, []
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
        self.websocket.publish("ros.provider.diagnostics",
                               json.dumps(self._toJsonDiagnostics(rosmsg), cls=SelfEncoder),)

    def getDiagnostics(self) -> DiagnosticArray:
        Log.info("interface: get diagnostics")
        rosmsg = self._monitor.get_diagnostics(0, 0)
        # copy message to the json structure
        return json.dumps(self._toJsonDiagnostics(rosmsg), cls=SelfEncoder)

    def getSystemInfo(self) -> SystemInformation:
        Log.info("interface: get system info")
        return json.dumps(SystemInformation(), cls=SelfEncoder)

    def getSystemEnv(self) -> SystemEnvironment:
        Log.info("interface: get system env")
        return json.dumps(SystemEnvironment(), cls=SelfEncoder)

    def rosCleanPurge(self) -> {bool, str}:
        Log.info("interface: ros_clean_purge")
        result = False
        message = ''
        try:
            screen.ros_clean()
            result = True
        except Exception as error:
            message = str(error)
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
                    # stop daemon after all other processes are killed
                    if (cmdStr.find('mas-daemon') == -1):
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

    def getLoggers(self, name: str) -> str:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.nodes.get_loggers] for '{name}'")
        loggerConfigs: List[LoggerConfig] = []
        service_name = '%s/get_loggers' % name
        get_logger = rospy.ServiceProxy(service_name, GetLoggers)
        resp = get_logger()
        for logger in resp.loggers:
            loggerConfigs.append(LoggerConfig(
                level=logger.level, name=logger.name))
        return json.dumps(loggerConfigs, cls=SelfEncoder)

    def setLoggerLevel(self, name: str, logger_json: List[LoggerConfig]) -> str:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.nodes.set_logger_level] for '{name}'")
        # Covert input dictionary into a proper python object
        loggers = json.loads(json.dumps(logger_json),
                             object_hook=lambda d: SimpleNamespace(**d))
        # request the current logger
        service_name_get = '%s/get_loggers' % name
        get_logger = rospy.ServiceProxy(service_name_get, GetLoggers)
        resp = get_logger()
        service_name_set = '%s/set_logger_level' % name
        for lRemote in loggers:
            # call set service only if new level
            doSet = True
            for lCurrent in resp.loggers:
                if lCurrent.name == lRemote.name and lCurrent.level == lRemote.level:
                    doSet = False
            if doSet:
                set_logger = rospy.ServiceProxy(
                    service_name_set, SetLoggerLevel)
                resultSet = set_logger(SetLoggerLevelRequest(
                    logger=lRemote.name, level=lRemote.level))
        return json.dumps({'result': True, 'message': ''}, cls=SelfEncoder)
