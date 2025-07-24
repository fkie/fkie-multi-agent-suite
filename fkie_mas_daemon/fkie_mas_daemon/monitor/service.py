# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from typing import List
import threading
import time

from rclpy.clock import Clock

import fkie_mas_daemon as nmd
from fkie_mas_pylib.settings import Settings
from fkie_mas_pylib.system.host import get_host_name
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from fkie_mas_daemon.time_helpers import rostime2float
from .cpu_load import CpuLoad
from .cpu_temperatur import CpuTemperatur
from .hdd_usage import HddUsage
from .mem_usage import MemUsage
from .net_load import NetLoad


class DiagnosticObj(DiagnosticStatus):

    def __init__(self, msg=DiagnosticStatus(), timestamp=0):
        self.msg = msg
        self.timestamp = timestamp

    def __eq__(self, item):
        if isinstance(item, DiagnosticObj):
            return self.msg.name == item.msg.name
        if isinstance(item, DiagnosticStatus):
            return self.msg.name == item.name

    def __ne__(self, item):
        return not (self == item)

    def __gt__(self, item):
        if isinstance(item, DiagnosticObj):
            return self.msg.name > item.msg.name
        if isinstance(item, DiagnosticStatus):
            return self.msg.name > item.name


class Service:

    DEBOUNCE_DIAGNOSTICS = 0.5

    def __init__(self, settings: Settings, callbackDiagnostics=None):
        self._clock = Clock()
        self._settings = settings
        self._mutex = threading.RLock()
        self._diagnostics = []  # DiagnosticObj
        self._update_last_ts = 0
        self._update_timer = None
        self._callbackDiagnostics = callbackDiagnostics
        self._sub_diag = None
        self._local_nodes: List[str] = []
        self._sub_diag_agg = nmd.ros_node.create_subscription(
            DiagnosticArray, '/diagnostics_agg', self._callback_diagnostics_agg, 10)
        self._sub_diag = nmd.ros_node.create_subscription(
            DiagnosticArray, 'diagnostics', self._callback_diagnostics, 10)
        hostname = get_host_name()

        self.sensors = []
        self.sensors.append(CpuLoad(hostname, 1.0, 0.9))
        self.sensors.append(CpuTemperatur(hostname, 1.0, 85.0))
        self.sensors.append(HddUsage(hostname, 30.0, 0.95))
        self.sensors.append(MemUsage(hostname, 1.0, 0.95))
        self.sensors.append(NetLoad(hostname, 1.0, 0.9))
        for sensor in self.sensors:
            self._settings.add_reload_listener(sensor.reload_parameter)
        self._settings.add_reload_listener(self.reload_parameter)

    def reload_parameter(self, settings: Settings):
        pass

    def update_local_node_names(self, local_nodes: List[str]):
        with self._mutex:
            self._local_nodes = local_nodes

    def _callback_diagnostics_agg(self, msg: DiagnosticArray):
        # aggregated diagnostics are stored
        with self._mutex:
            stamp = float(msg.header.stamp.sec) + \
                float(msg.header.stamp.nanosec) / 1000000000.0
            stamp = time.time()
            for status in msg.status:
                try:
                    idx = self._diagnostics.index(status)
                    diag_obj = self._diagnostics[idx]
                    diag_obj.msg = status
                    diag_obj.timestamp = stamp
                except Exception:
                    diag_obj = DiagnosticObj(status, stamp)
                    diag_obj.timestamp = stamp
                    self._diagnostics.append(diag_obj)
            if self._callbackDiagnostics:
                now = time.time()
                if now - self._update_last_ts > self.DEBOUNCE_DIAGNOSTICS:
                    # at the first message, send immediately
                    self._update_last_ts = now
                    self._callbackDiagnostics(msg)
                elif self._update_timer is None:
                    # start the timer
                    self._update_timer = threading.Timer(
                        self.DEBOUNCE_DIAGNOSTICS, self._publish_diagnostics)
                    self._update_timer.start()

    def _callback_diagnostics(self, msg: DiagnosticArray):
        # diagnostic messages are forwarded immediately and not stored
        # forward only status message which has name of local nodes.
        if self._callbackDiagnostics:
            filteredMsg = DiagnosticArray()
            filteredMsg.header = msg.header
            with self._mutex:
                for status in msg.status:
                    if status.name in self._local_nodes:
                        filteredMsg.status.append(status)
            if len(filteredMsg.status) > 0:
                self._callbackDiagnostics(filteredMsg)

    def _publish_diagnostics(self):
        diags = self.get_diagnostics(0, self._update_last_ts)
        if self._callbackDiagnostics and len(diags.status) > 0:
            self._callbackDiagnostics(diags)
        self._update_last_ts = time.time()
        self._update_timer.cancel()
        self._update_timer = None

    def get_system_diagnostics(self, filter_level: int, filter_ts: float = 0):
        result = DiagnosticArray()
        with self._mutex:
            now = self._clock.now()
            result.header.stamp = self._clock.now().to_msg()
            for sensor in self.sensors:
                try:
                    diag_msg = sensor.last_state(
                        rostime2float(now), filter_level, filter_ts)
                    if diag_msg is not None:
                        result.status.append(diag_msg)
                except:
                    import traceback
                    print(traceback.print_exc())
        return result

    def get_diagnostics(self, filter_level: int, filter_ts: float = 0):
        result = DiagnosticArray()
        # rospy.Time.from_sec(time.time())
        result.header.stamp = self._clock.now().to_msg()
        with self._mutex:
            for diag_obj in self._diagnostics:
                if diag_obj.timestamp >= filter_ts:
                    if int.from_bytes(diag_obj.msg.level, byteorder='big') >= filter_level:
                        # if diag_obj.msg.level >= filter_level:
                        result.status.append(diag_obj.msg)
        return result

    def stop(self):
        with self._mutex:
            for sensor in self.sensors:
                sensor.cancel_timer()
            # destroy subscription to avoid runtime warnings
            if self._sub_diag is not None:
                nmd.ros_node.destroy_subscription(self._sub_diag)
                self._sub_diag = None
