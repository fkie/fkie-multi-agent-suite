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


import rclpy
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

    DEBOUNCE_DIAGNOSTICS = 1.0

    def __init__(self, settings: Settings, callbackDiagnostics=None):
        self._clock = Clock()
        self._settings = settings
        self._mutex = threading.RLock()
        self._diagnostics = []  # DiagnosticObj
        self._update_last_ts = 0
        self._update_timer = None
        self._callbackDiagnostics = callbackDiagnostics
        self.use_diagnostics_agg = settings.param(
            'global/use_diagnostics_agg', True)
        self._sub_diag = None
        if self.use_diagnostics_agg:
            self._sub_diag = nmd.ros_node.create_subscription(
                DiagnosticArray, '/diagnostics_agg', self._callback_diagnostics, 100)
        else:
            self._sub_diag = nmd.ros_node.create_subscription(
                DiagnosticArray, '/diagnostics', self._callback_diagnostics, 100)
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
        value = settings.param('global/use_diagnostics_agg', False)
        if value != self.use_diagnostics_agg:
            if self._sub_diag is not None:
                nmd.ros_node.destroy_subscription(self._sub_diag)
                self._sub_diag = None
            if value:
                self._sub_diag = nmd.ros_node.create_subscription(
                    DiagnosticArray, '/diagnostics_agg', self._callback_diagnostics, 100)
            else:
                self._sub_diag = nmd.ros_node.create_subscription(
                    DiagnosticArray, '/diagnostics', self._callback_diagnostics, 100)
            self.use_diagnostics_agg = value

    def _callback_diagnostics(self, msg: DiagnosticArray):
        # TODO: update diagnostics
        with self._mutex:
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000.0
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
                if stamp - self._update_last_ts > self.DEBOUNCE_DIAGNOSTICS * 2.0:
                    # at the first message, send immediately
                    self._update_last_ts = stamp
                    self._callbackDiagnostics(msg)
                elif self._update_timer is None:
                    # start the timer
                    self._update_timer = threading.Timer(self.DEBOUNCE_DIAGNOSTICS, self._publish_diagnostics)

    def _publish_diagnostics(self):
        diags = self.get_diagnostics(0, self._update_last_ts)
        if self._callbackDiagnostics and len(diags.status) > 0:
            self._callbackDiagnostics(diags)
        self._update_last_ts = time.time()
        self._update_timer.cancel()
        self._update_timer = None

    def get_system_diagnostics(self, filter_level: list = [], filter_ts: float = 0):
        result = DiagnosticArray()
        with self._mutex:
            now = self._clock.now()
            result.header.stamp = self._clock.now().to_msg()
            for sensor in self.sensors:
                diag_msg = sensor.last_state(
                    rostime2float(now), filter_level, filter_ts)
                if diag_msg is not None:
                    result.status.append(diag_msg)
        return result

    def get_diagnostics(self, filter_level: list = [], filter_ts: float = 0):
        result = DiagnosticArray()
        # rospy.Time.from_sec(time.time())
        result.header.stamp = self._clock.now().to_msg()
        with self._mutex:
            for diag_obj in self._diagnostics:
                if diag_obj.timestamp >= filter_ts:
                    #                    if int.from_bytes(diag_obj.msg.level, byteorder='big') >= filter_level:
                    if diag_obj.msg.level in filter_level:
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
