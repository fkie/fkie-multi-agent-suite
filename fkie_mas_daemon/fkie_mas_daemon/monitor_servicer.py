# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from typing import Dict
from typing import List

import json
import os
import psutil
import shutil
import signal
import threading
from fkie_mas_daemon.monitor.service import Service
from fkie_mas_pylib.interface.runtime_interface import DiagnosticArray
from fkie_mas_pylib.interface.runtime_interface import DiagnosticStatus
from fkie_mas_pylib.interface.runtime_interface import SystemEnvironment
from fkie_mas_pylib.interface.runtime_interface import SystemInformation
from fkie_mas_pylib.interface.runtime_interface import SystemWarningGroup
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import process
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.defines import LOG_PATH
from fkie_mas_pylib.defines import SETTINGS_PATH
from fkie_mas_pylib import names
from fkie_mas_pylib.websocket.server import WebSocketServer


class MonitorServicer:
    def __init__(self, settings, websocket: WebSocketServer):
        Log.info("Create monitor servicer")
        self._killTimer = None
        self._monitor = Service(settings, self.diagnosticsCbPublisher)
        self.websocket = websocket
        self._warning_groups: Dict[str, SystemWarningGroup] = {}
        websocket.register("ros.provider.get_system_info", self.getSystemInfo)
        websocket.register("ros.provider.get_system_env", self.getSystemEnv)
        websocket.register("ros.provider.get_warnings", self.getProviderWarnings)
        websocket.register("ros.provider.get_diagnostics", self.getDiagnostics)
        websocket.register("ros.provider.ros_clean_purge", self.rosCleanPurge)
        websocket.register("ros.provider.shutdown", self.rosShutdown)
        websocket.register("ros.process.find_node", self.findNode)
        websocket.register("ros.process.kill", self.killProcess)

    def stop(self):
        self._monitor.stop()

    def update_warning_groups(self, warnings: List[SystemWarningGroup]):
        updated = False
        for group in warnings:
            if group.id not in self._warning_groups:
                updated = True
                self._warning_groups[group.id] = group.copy()
            elif not self._warning_groups[group.id] == group:
                updated = True
                self._warning_groups[group.id] = group.copy()
        if updated:
            count_warnings = 0
            for wg in self._warning_groups.values():
                count_warnings += len(wg.warnings)
            Log.info(f"{self.__class__.__name__}: ros.provider.warnings with {count_warnings} warnings in {len(self._warning_groups)} groups")
            self.websocket.publish('ros.provider.warnings', list(self._warning_groups.values()))

    def update_local_node_names(self, local_nodes: List[str]):
        self._monitor.update_local_node_names(local_nodes)

    def getSystemInfo(self) -> SystemInformation:
        Log.info(f"{self.__class__.__name__}: request: get system info")
        return json.dumps(SystemInformation(), cls=SelfEncoder)

    def getSystemEnv(self) -> SystemEnvironment:
        Log.info(f"{self.__class__.__name__}: request: get system env")
        return json.dumps(SystemEnvironment(), cls=SelfEncoder)

    def getProviderWarnings(self) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.provider.get_warnings]")
        return json.dumps(list(self._warning_groups.values()), cls=SelfEncoder)

    def _toJsonDiagnostics(self, ros_msg):
        cbMsg = DiagnosticArray(
            timestamp=float(ros_msg.header.stamp.sec)
            + float(ros_msg.header.stamp.nanosec) / 1000000000.0, status=[]
        )
        for sensor in ros_msg.status:
            values = []
            for v in sensor.values:
                values.append(DiagnosticStatus.KeyValue(v.key, v.value))
            level = 0
            try:
                level = int.from_bytes(sensor.level, byteorder='big')
            except:
                pass
            status = DiagnosticStatus(
                level, sensor.name, sensor.message, sensor.hardware_id, values
            )
            cbMsg.status.append(status)
        return cbMsg

    def diagnosticsCbPublisher(self, ros_msg):
        self.websocket.publish("ros.provider.diagnostics",
                               json.dumps(self._toJsonDiagnostics(ros_msg), cls=SelfEncoder),)

    def getDiagnostics(self) -> DiagnosticArray:
        Log.info(f"{self.__class__.__name__}: request: get diagnostics")
        ros_msg = self._monitor.get_diagnostics(0, 0)
        ros_msg.status.extend(self._monitor.get_system_diagnostics(0, 0.0).status)
        # copy message to the JSON structure
        return json.dumps(self._toJsonDiagnostics(ros_msg), cls=SelfEncoder)

    def rosCleanPurge(self) -> {bool, str}:
        Log.info(f"{self.__class__.__name__}: request: ros_clean_purge")
        result = False
        if os.path.exists(LOG_PATH):
            try:
                shutil.rmtree(LOG_PATH)
                os.makedirs(LOG_PATH)
                result = True
                message = f"Purging ROS node logs from {LOG_PATH}"
            except Exception as e:
                message = f"{e}"
        return json.dumps({"result": result, "message": message}, cls=SelfEncoder)

    def rosShutdown(self) -> {bool, str}:
        Log.info(f"{self.__class__.__name__}: ros.provider.shutdown")
        result = False
        message = ''
        procs = []
        screen_child_ids = []
        try:
            for ps_it in psutil.process_iter():
                try:
                    cmdStr = ' '.join(ps_it.cmdline())
                    if cmdStr.find(SETTINGS_PATH) > -1:
                        # ignore mas daemon pid to kill it last
                        if (cmdStr.find('mas-daemon') == -1):
                            found_pid, _found_name, _parents2kill = process.get_child_pid(ps_it.pid)
                            procs.append(ps_it)
                            ps_it.terminate()
                            if found_pid > -1:
                                # store child process of the screen we found using SETTINGS_PATH for later kill
                                screen_child_ids.append(found_pid)
                except (psutil.ZombieProcess, psutil.NoSuchProcess):
                    # ignore errors because of zombie processes or non-existent (terminated child?) processes
                    pass
                except Exception as error:
                    import traceback
                    print(traceback.format_exc())
            # kill child process of the screen we found using SETTINGS_PATH
            for pid in screen_child_ids:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    # ignore errors for non-existent processes, they were already terminated when the parent screen process was stopped
                    pass
                except Exception as error:
                    import traceback
                    print(traceback.format_exc())
            gone, alive = psutil.wait_procs(procs, timeout=3)
            for p in alive:
                p.kill()
            self._killTimer = threading.Timer(1.0, self._killSelf)
            self._killTimer.start()
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

    def findNode(self, name: str) -> str:
        Log.info(f"{self.__class__.__name__}: find node '{name}'")
        success = False
        ns = names.namespace(name).rstrip('/')
        basename = names.basename(name)
        count = 0
        processes = []
        for process in psutil.process_iter():
            count += 1
            cmd_line = ' '.join(process.cmdline())
            if cmd_line.find(f"__node:={basename}") > -1 and (not ns or cmd_line.find(f"__ns:={ns}") > -1):
                ps = {"pid": process.pid, "cmdLine": cmd_line}
                return json.dumps({"result": True, "message": "", "processes": [ps]}, cls=SelfEncoder)
            elif cmd_line.find("/ros2 ") > -1:
                processes.append({"pid": process.pid, "cmdLine": cmd_line})
        if len(processes) > 0:
            return json.dumps({"result": True, "message": "", "processes": processes}, cls=SelfEncoder)
        return json.dumps({"result": False, "message": f"node {name} not found"})

    def killProcess(self, pid: int, sig=signal.SIGTERM):
        Log.info(f"{self.__class__.__name__}: kill process '{pid}' with sig: {sig}")
        if psutil.pid_exists(pid):
            os.kill(pid, sig)
        if sig != signal.SIGKILL:
            killTimer = threading.Timer(1.0, self.killProcess, args=(pid, signal.SIGKILL))
            killTimer.start()
