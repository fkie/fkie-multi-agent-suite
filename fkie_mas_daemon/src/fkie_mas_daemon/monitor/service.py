# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import rospy
import socket
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
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

    def __init__(self, settings, callbackDiagnostics=None):
        self._settings = settings
        self._mutex = threading.RLock()
        self._diagnostics = []  # DiagnosticObj
        self._callbackDiagnostics = callbackDiagnostics
        self.use_diagnostics_agg = settings.param("global/use_diagnostics_agg", True)
        self._sub_diag_agg = None
        self._sub_diag = None
        self._update_last_ts = 0
        self._update_timer = None
        if self.use_diagnostics_agg:
            self._sub_diag_agg = rospy.Subscriber(
                "/diagnostics_agg", DiagnosticArray, self._callback_diagnostics
            )
        else:
            self._sub_diag = rospy.Subscriber(
                "/diagnostics", DiagnosticArray, self._callback_diagnostics
            )
        hostname = socket.gethostname()

        self.sensors = []
        self.sensors.append(CpuLoad(hostname, 1.0, 0.9))
        self.sensors.append(CpuTemperatur(hostname, 1.0, 85.0))
        self.sensors.append(HddUsage(hostname, 30.0, 0.95))
        self.sensors.append(MemUsage(hostname, 1.0, 0.95))
        self.sensors.append(NetLoad(hostname, 1.0, 0.9))
        for sensor in self.sensors:
            self._settings.add_reload_listener(sensor.reload_parameter)
        self._settings.add_reload_listener(self.reload_parameter)

    def reload_parameter(self, settings):
        value = settings.param("global/use_diagnostics_agg", True)
        if value != self.use_diagnostics_agg:
            if self._sub_diag is not None:
                self._sub_diag.unregister()
                self._sub_diag = None
            if self._sub_diag_agg is not None:
                self._sub_diag_agg.unregister()
                self._sub_diag_agg = None
            if value:
                self._sub_diag_agg = rospy.Subscriber(
                    "/diagnostics_agg", DiagnosticArray, self._callback_diagnostics
                )
            else:
                self._sub_diag = rospy.Subscriber(
                    "/diagnostics", DiagnosticArray, self._callback_diagnostics
                )
            self.use_diagnostics_agg = value

    def _callback_diagnostics(self, msg):
        # TODO: update diagnostics
        with self._mutex:
            now = time.time()
            stamp = msg.header.stamp.to_sec()
            for status in msg.status:
                try:
                    idx = self._diagnostics.index(status)
                    diag_obj = self._diagnostics[idx]
                    diag_obj.msg = status
                    diag_obj.timestamp = stamp
                except Exception:
                    diag_obj = DiagnosticObj(status, stamp)
                    self._diagnostics.append(diag_obj)
            if self._callbackDiagnostics:
                if now - self._update_last_ts > self.DEBOUNCE_DIAGNOSTICS * 2.0:
                    # at the first message, send immediately
                    self._update_last_ts = now
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

    def get_system_diagnostics(self, filter_level=0, filter_ts=0):
        result = DiagnosticArray()
        with self._mutex:
            nowsec = time.time()
            result.header.stamp = rospy.Time.from_sec(nowsec)
            for sensor in self.sensors:
                diag_msg = sensor.last_state(nowsec, filter_level, filter_ts)
                if diag_msg is not None:
                    result.status.append(diag_msg)
        return result

    def get_diagnostics(self, filter_level=0, filter_ts=0):
        result = DiagnosticArray()
        result.header.stamp = rospy.Time.from_sec(time.time())
        with self._mutex:
            for diag_obj in self._diagnostics:
                if diag_obj.timestamp >= filter_ts:
                    if diag_obj.msg.level >= filter_level:
                        result.status.append(diag_obj.msg)
        return result

    def stop(self):
        with self._mutex:
            for sensor in self.sensors:
                sensor.stop()
