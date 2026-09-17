# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import json
import os
import re
import shutil
import signal
import threading
import time
from collections.abc import Sequence

import psutil
from fkie_mas_pylib.defines import LOG_PATH, SETTINGS_PATH
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.runtime_interface import (
    DiagnosticArray,
    DiagnosticStatus,
    SystemEnvironment,
    SystemInformation,
    SystemWarningGroup,
)
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import process, screen
from fkie_mas_pylib.websocket.server import WebSocketServer

from fkie_mas_daemon.monitor.service import Service
from fkie_mas_pylib import names


class MonitorServicer:
    WARNING_LIFETIME_SEC = 60

    # topic used to publish the periodically collected system diagnostics
    SYSTEM_DIAGNOSTICS_URI = "ros.provider.system_diagnostics"
    # topic used to publish the provider warnings
    PROVIDER_WARNINGS_URI = "ros.provider.warnings"
    # default period of the background measurement in seconds
    DEFAULT_SYSTEM_DIAGNOSTICS_INTERVAL = 1.0
    # minimum period to avoid a busy loop caused by an invalid parameter
    MIN_SYSTEM_DIAGNOSTICS_INTERVAL = 0.1
    # period used to check for new subscribers while the thread is idle
    SUBSCRIPTION_CHECK_INTERVAL = 2.0
    # default patterns of processes which are not treated as ros2 processes
    DEFAULT_ROS2_EXCLUDE = ("colcon", "cmake", "CMakeFiles")

    def __init__(self, settings, websocket: WebSocketServer):
        Log.info("Create monitor servicer")
        self._killTimer = None
        self._settings = settings
        self._monitor = Service(settings, self.diagnosticsCbPublisher)
        self.websocket = websocket
        # protects the warning groups, they are updated from different threads
        self._warnings_lock = threading.RLock()
        self._warning_groups: dict[str, SystemWarningGroup] = {}
        # state of the periodic system diagnostics publisher
        self._sysdiag_interval = self.DEFAULT_SYSTEM_DIAGNOSTICS_INTERVAL
        self._sysdiag_stop_event = threading.Event()
        # set if subscribers changed, a parameter was reloaded or on shutdown
        self._sysdiag_wakeup = threading.Event()
        self._sysdiag_thread: threading.Thread | None = None
        # True while the last published message contained at least one warning
        self._sysdiag_had_warning = False
        # signature of the warnings of the last published message;
        # empty set = no active warnings
        self._sysdiag_warning_state: frozenset = frozenset()
        # cache for the compiled exclude patterns of isRos2Process()
        self._exclude_patterns: dict[tuple[str, ...], list[re.Pattern]] = {}
        websocket.register("ros.provider.get_system_info", self.getSystemInfo)
        websocket.register("ros.provider.get_system_env", self.getSystemEnv)
        websocket.register("ros.provider.get_warnings", self.getProviderWarnings)
        websocket.register("ros.provider.get_system_diagnostics", self.getSystemDiagnostics)
        websocket.register("ros.provider.get_diagnostics", self.getDiagnostics)
        websocket.register("ros.provider.ros_clean_purge", self.rosCleanPurge)
        websocket.register("ros.provider.shutdown", self.rosShutdown)
        websocket.register("ros.process.find_node", self.findNode)
        websocket.register("ros.process.kill", self.killProcess)
        self._settings.add_reload_listener(self.reload_parameter)
        # register the listener before the thread starts, so no change is lost
        websocket.add_subscription_listener(self._on_subscription_changed)
        self._start_system_diagnostics_thread()

    def _on_subscription_changed(self, uri: str, count: int) -> None:
        # wake up the loop instead of waiting for the next poll cycle
        if uri == self.SYSTEM_DIAGNOSTICS_URI:
            Log.debug(f"{self.__class__.__name__}: subscribers for {uri}: {count}")
            self._sysdiag_wakeup.set()

    def reload_parameter(self, settings):
        # period of the background system diagnostics measurement
        interval = settings.param("sysmon/System/diagnostics_interval", self._sysdiag_interval)
        try:
            interval = float(interval)
        except (TypeError, ValueError):
            Log.warn(
                f"{self.__class__.__name__}: invalid diagnostics_interval '{interval}', "
                f"use default {self.DEFAULT_SYSTEM_DIAGNOSTICS_INTERVAL}s"
            )
            interval = self.DEFAULT_SYSTEM_DIAGNOSTICS_INTERVAL
        # avoid a busy loop caused by an invalid parameter
        new_interval = max(self.MIN_SYSTEM_DIAGNOSTICS_INTERVAL, interval)
        if new_interval != self._sysdiag_interval:
            self._sysdiag_interval = new_interval
            Log.info(f"{self.__class__.__name__}: system diagnostics interval: {new_interval}s")
            # apply the new interval without waiting for the current cycle
            self._sysdiag_wakeup.set()

    def stop(self):
        # remove the listener first, it uses the events of this instance
        try:
            self.websocket.remove_subscription_listener(self._on_subscription_changed)
        except Exception as error:
            Log.debug(f"{self.__class__.__name__}: can not remove subscription listener: {error}")
        self._stop_system_diagnostics_thread()
        # cancel a pending self kill timer
        if self._killTimer is not None:
            self._killTimer.cancel()
            self._killTimer = None
        self._monitor.stop()

    def _start_system_diagnostics_thread(self):
        """
        Starts the background thread which publishes the system diagnostics.
        The thread does not measure anything as long as nobody is subscribed to
        SYSTEM_DIAGNOSTICS_URI.
        """
        if self._sysdiag_thread is not None and self._sysdiag_thread.is_alive():
            return
        self._sysdiag_stop_event.clear()
        self._sysdiag_wakeup.clear()
        self._sysdiag_thread = threading.Thread(
            target=self._system_diagnostics_loop, name="system_diagnostics", daemon=True
        )
        self._sysdiag_thread.start()

    def _stop_system_diagnostics_thread(self):
        """
        Stops the background thread and waits until it has finished.
        """
        self._sysdiag_stop_event.set()
        # the loop waits on the wakeup event, so it returns immediately
        self._sysdiag_wakeup.set()
        thread = self._sysdiag_thread
        self._sysdiag_thread = None
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
            if thread.is_alive():
                Log.warn(f"{self.__class__.__name__}: system diagnostics thread did not stop")

    def _count_subscriptions(self, uri: str) -> int:
        # the websocket server reports the count of remote and local subscribers
        try:
            return self.websocket.subscriptions(uri)
        except Exception as error:
            Log.debug(f"{self.__class__.__name__}: can not determine subscriptions for {uri}: {error}")
            return 0

    def _system_diagnostics_loop(self):
        """
        Periodically collects the system diagnostics while at least one client
        is subscribed.
        """
        while not self._sysdiag_stop_event.is_set():
            # clear before the check, so a change during the cycle is not lost
            self._sysdiag_wakeup.clear()
            if self._count_subscriptions(self.SYSTEM_DIAGNOSTICS_URI) <= 0:
                # nobody is listening: reset the state, so the next subscriber
                # gets the current warnings once and only wait for subscribers
                self._sysdiag_warning_state = frozenset()
                self._sysdiag_wakeup.wait(self.SUBSCRIPTION_CHECK_INTERVAL)
                continue
            ts_start = time.monotonic()
            try:
                self._publish_system_diagnostics()
            except Exception as error:
                Log.warn(f"{self.__class__.__name__}: error while publishing system diagnostics: {error}")
            # compensate the runtime of the measurement to avoid a drift
            wait = self._sysdiag_interval - (time.monotonic() - ts_start)
            if wait > 0.0:
                self._sysdiag_wakeup.wait(wait)
        Log.debug(f"{self.__class__.__name__}: system diagnostics loop stopped")

    def _warning_signature(self, ros_msg) -> frozenset:
        """
        Builds a comparable signature of all currently active warnings.
        The message text is intentionally ignored, it usually contains
        measured values which change in every cycle.
        """
        signature = set()
        for status in ros_msg.status:
            level = self._diagnostic_level(status.level)
            if level > 0:
                signature.add((status.name, status.hardware_id, level))
        return frozenset(signature)

    def _publish_system_diagnostics(self):
        """
        Requests the current sensor states and publishes them only if the set of
        active warnings changed, i.e. a warning appeared, disappeared or changed
        its level. Nothing is published while the warning state is unchanged
        (also not if there is no warning at all).
        """
        ros_msg = self._monitor.get_system_diagnostics(0, 0)
        signature = self._warning_signature(ros_msg)
        if signature == self._sysdiag_warning_state:
            # no warning appeared, disappeared or changed its level
            return
        Log.debug(
            f"{self.__class__.__name__}: warning state changed "
            f"({len(self._sysdiag_warning_state)} -> {len(signature)}), publish {self.SYSTEM_DIAGNOSTICS_URI}"
        )
        self._sysdiag_warning_state = signature
        self.websocket.publish(
            self.SYSTEM_DIAGNOSTICS_URI, json.dumps(self._toJsonDiagnostics(ros_msg), cls=SelfEncoder)
        )

    @staticmethod
    def _diagnostic_level(level) -> int:
        """
        Converts the level of a ROS DiagnosticStatus to an int. Depending on the
        rclpy version the 'byte' field is reported as bytes or as int.
        """
        if isinstance(level, int):
            return level
        if isinstance(level, (bytes, bytearray)):
            return int.from_bytes(level, byteorder="big")
        try:
            return int(level)
        except (TypeError, ValueError):
            return 0

    def _has_subscribers(self, uri: str) -> bool:
        """
        Returns True if at least one local or remote client is subscribed to uri.
        """
        return self._count_subscriptions(uri) > 0

    def _publish_if_subscribed(self, uri: str, payload_factory) -> bool:
        """
        Publishes the result of payload_factory() only if at least one client is
        subscribed to uri. The payload is created lazily, so the copy of the
        warning groups and the JSON serialization are skipped if nobody listens.
        """
        if not self._has_subscribers(uri):
            Log.debug(f"{self.__class__.__name__}: skip publish {uri}, no subscribers")
            return False
        self.websocket.publish(uri, payload_factory())
        return True

    def remove_warning_group(self, group: SystemWarningGroup | str):
        """
        Removes a warning group. Accepts the group object as well as its id.
        """
        # the groups are stored by id, a group object would never match
        group_id = group.id if isinstance(group, SystemWarningGroup) else group
        with self._warnings_lock:
            if group_id not in self._warning_groups:
                return
            del self._warning_groups[group_id]
            # do not build the list if nobody is subscribed
            if not self._has_subscribers(self.PROVIDER_WARNINGS_URI):
                Log.debug(f"{self.__class__.__name__}: skip publish {self.PROVIDER_WARNINGS_URI}, no subscribers")
                return
            groups = list(self._warning_groups.values())
        self.websocket.publish(self.PROVIDER_WARNINGS_URI, json.dumps(groups, cls=SelfEncoder))

    def update_warning_groups(self, warnings: list[SystemWarningGroup]):
        updated = False
        groups = []
        count_warnings = 0
        with self._warnings_lock:
            for group in warnings:
                if group.id not in self._warning_groups:
                    updated = True
                    self._warning_groups[group.id] = group.copy()
                elif not self._warning_groups[group.id] == group:
                    updated = True
                    new_group = group.copy()
                    now = time.time()
                    # add only newest messages
                    for ogw in self._warning_groups[group.id].warnings:
                        if now - ogw.timestamp < self.WARNING_LIFETIME_SEC:
                            new_group.warnings.append(ogw)
                    self._warning_groups[group.id] = new_group
            if not updated:
                # the state did not change, nothing to publish
                return
            # the internal state is always updated, only the publish is skipped
            if not self._has_subscribers(self.PROVIDER_WARNINGS_URI):
                Log.debug(f"{self.__class__.__name__}: skip publish {self.PROVIDER_WARNINGS_URI}, no subscribers")
                return
            groups = list(self._warning_groups.values())
            for wg in groups:
                count_warnings += len(wg.warnings)
        Log.info(
            f"{self.__class__.__name__}: {self.PROVIDER_WARNINGS_URI} with {count_warnings} warnings in {len(groups)} groups"
        )
        # publish outside of the lock, it performs network io
        self.websocket.publish(self.PROVIDER_WARNINGS_URI, json.dumps(groups, cls=SelfEncoder))

    def diagnosticsCbPublisher(self, ros_msg):
        # skip the JSON conversion of the complete diagnostics array if nobody listens
        self._publish_if_subscribed(
            "ros.provider.diagnostics", lambda: json.dumps(self._toJsonDiagnostics(ros_msg), cls=SelfEncoder)
        )

    def update_local_node_names(self, local_nodes: list[str]):
        self._monitor.update_local_node_names(local_nodes)

    def getSystemInfo(self) -> str:
        Log.info(f"{self.__class__.__name__}: request: get system info")
        return json.dumps(SystemInformation(), cls=SelfEncoder)

    def getSystemEnv(self) -> str:
        Log.info(f"{self.__class__.__name__}: request: get system env")
        return json.dumps(SystemEnvironment(), cls=SelfEncoder)

    def getProviderWarnings(self) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.provider.get_warnings]")
        now = time.time()
        with self._warnings_lock:
            # build a new dict, do not modify the dict while iterating it
            current: dict[str, SystemWarningGroup] = {}
            for group_id, group in self._warning_groups.items():
                new_group = SystemWarningGroup(group_id)
                # add only newest messages
                for ogw in group.warnings:
                    if now - ogw.timestamp < self.WARNING_LIFETIME_SEC:
                        new_group.warnings.append(ogw)
                current[group_id] = new_group
            self._warning_groups = current
            groups = list(current.values())
        return json.dumps(groups, cls=SelfEncoder)

    def _toJsonDiagnostics(self, ros_msg):
        cbMsg = DiagnosticArray(
            timestamp=float(ros_msg.header.stamp.sec) + float(ros_msg.header.stamp.nanosec) / 1000000000.0, status=[]
        )
        for sensor in ros_msg.status:
            values = []
            for v in sensor.values:
                values.append(DiagnosticStatus.KeyValue(v.key, v.value))
            # the level is reported as bytes or int, depending on rclpy version
            level = self._diagnostic_level(sensor.level)
            status = DiagnosticStatus(level, sensor.name, sensor.message, sensor.hardware_id, values)
            cbMsg.status.append(status)
        return cbMsg

    def getSystemDiagnostics(self) -> str:
        Log.info(f"{self.__class__.__name__}: request: get system diagnostics")
        # runs once on request, independent of the background thread
        ros_msg = self._monitor.get_system_diagnostics(0, 0)
        # copy message to the JSON structure
        return json.dumps(self._toJsonDiagnostics(ros_msg), cls=SelfEncoder)

    def getDiagnostics(self) -> str:
        Log.info(f"{self.__class__.__name__}: request: get diagnostics")
        ros_msg = self._monitor.get_diagnostics(0, 0)
        # copy message to the JSON structure
        return json.dumps(self._toJsonDiagnostics(ros_msg), cls=SelfEncoder)

    def rosCleanPurge(self) -> str:
        Log.info(f"{self.__class__.__name__}: request: ros_clean_purge")
        result = False
        # initialize the message, it was unbound if LOG_PATH does not exist
        message = f"{LOG_PATH} does not exist"
        if os.path.exists(LOG_PATH):
            try:
                shutil.rmtree(LOG_PATH)
                os.makedirs(LOG_PATH)
                result = True
                message = f"Purging ROS node logs from {LOG_PATH}"
            except Exception as e:
                message = f"{e}"
        return json.dumps({"result": result, "message": message}, cls=SelfEncoder)

    def _get_exclude_patterns(self, exclude: Sequence[str] | None) -> list[re.Pattern]:
        """
        Returns the compiled exclude patterns. The result is cached, the regex
        is compiled once per process iteration and not per process.
        """
        use_exclude = tuple(exclude) if exclude else self.DEFAULT_ROS2_EXCLUDE
        if use_exclude in self._exclude_patterns:
            return self._exclude_patterns[use_exclude]
        patterns: list[re.Pattern] = []
        for ex in use_exclude:
            try:
                patterns.append(re.compile(ex))
            except re.error as e:
                Log.warn(f"invalid regex pattern '{ex}': {e}")
        self._exclude_patterns[use_exclude] = patterns
        return patterns

    def isRos2Process(self, cmd: str, exclude: list[str] = None) -> bool:
        # check the cheap condition first, the regex is only needed for ros2 processes
        if "ros2" not in cmd:
            return False
        patterns = self._get_exclude_patterns(exclude)
        return not any(p.search(cmd) for p in patterns)

    def rosShutdown(self, killRos2: bool = False, exclude: list[str] = None) -> str:
        Log.info(f"{self.__class__.__name__}: ros.provider.shutdown; killRos2: {killRos2}")
        result = False
        message = ""
        procs = []
        screen_child_ids = []
        try:
            for ps_it in psutil.process_iter():
                try:
                    cmdStr = " ".join(ps_it.cmdline())
                    if cmdStr.find(SETTINGS_PATH) > -1:
                        # ignore mas daemon pid to kill it last
                        if "mas-daemon" not in cmdStr:
                            found_pid, _found_name, _parents2kill = process.get_child_pid(ps_it.pid)
                            procs.append(ps_it)
                            ps_it.terminate()
                            if found_pid > -1:
                                # store child process of the screen we found using SETTINGS_PATH for later kill
                                screen_child_ids.append(found_pid)
                    elif killRos2 and self.isRos2Process(cmdStr, exclude) and "mas-daemon" not in cmdStr:
                        ps_it.terminate()
                        procs.append(ps_it)
                except (psutil.ZombieProcess, psutil.NoSuchProcess, psutil.AccessDenied):
                    # ignore errors because of zombie processes, non-existent
                    # (terminated child?) or inaccessible processes
                    pass
                except Exception as error:
                    Log.warn(f"{self.__class__.__name__}: error while terminating process: {error}")
            # kill child process of the screen we found using SETTINGS_PATH
            for pid in screen_child_ids:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    # ignore errors for non-existent processes, they were already terminated when the parent screen process was stopped
                    pass
                except Exception as error:
                    Log.warn(f"{self.__class__.__name__}: error while killing {pid}: {error}")
            _gone, alive = psutil.wait_procs(procs, timeout=3)
            for p in alive:
                try:
                    p.kill()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    # the process is already gone or not accessible
                    pass
            self._killTimer = threading.Timer(1.0, self._killSelf)
            # do not block the interpreter exit
            self._killTimer.daemon = True
            self._killTimer.start()
            result = True
        except Exception as error:
            Log.warn(f"{self.__class__.__name__}: error on ros shutdown: {error}")
            message = str(error)
        try:
            screen.wipe()
        except Exception as error:
            Log.debug(f"{self.__class__.__name__}: error while wipe screens: {error}")
        # use string keys, the variables were used as keys before
        return json.dumps({"result": result, "message": message}, cls=SelfEncoder)

    def _killSelf(self, pidList=None, sig=signal.SIGTERM):
        # avoid a mutable default argument
        for pid in pidList or []:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                # the process terminated in the meantime
                pass
        os.kill(os.getpid(), signal.SIGINT)

    def findNode(self, name: str) -> str:
        Log.info(f"{self.__class__.__name__}: find node '{name}'")
        ns = names.namespace(name).rstrip("/")
        basename = names.basename(name)
        processes = []
        for ps_it in psutil.process_iter():
            try:
                cmd_line = " ".join(ps_it.cmdline())
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                # the process disappeared or is not accessible
                continue
            if cmd_line.find(f"__node:={basename}") > -1 and (not ns or cmd_line.find(f"__ns:={ns}") > -1):
                ps = {"pid": ps_it.pid, "cmdLine": cmd_line}
                return json.dumps({"result": True, "message": "", "processes": [ps]}, cls=SelfEncoder)
            elif cmd_line.find("/ros2 ") > -1:
                processes.append({"pid": ps_it.pid, "cmdLine": cmd_line})
        if len(processes) > 0:
            return json.dumps({"result": True, "message": "", "processes": processes}, cls=SelfEncoder)
        return json.dumps({"result": False, "message": f"node {name} not found", "processes": []}, cls=SelfEncoder)

    def killProcess(self, pid: int, sig=signal.SIGTERM):
        Log.info(f"{self.__class__.__name__}: kill process '{pid}' with sig: {sig}")
        if psutil.pid_exists(pid):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                # the process terminated in the meantime
                return
            except PermissionError as error:
                Log.warn(f"{self.__class__.__name__}: can not kill {pid}: {error}")
                return
        elif sig == signal.SIGKILL:
            # nothing to do, the process is already gone
            return
        if sig != signal.SIGKILL:
            killTimer = threading.Timer(1.0, self.killProcess, args=(pid, signal.SIGKILL))
            killTimer.daemon = True
            killTimer.start()
