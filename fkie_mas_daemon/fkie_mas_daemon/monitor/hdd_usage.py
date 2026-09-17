# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import time

import psutil
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from fkie_mas_pylib.defines import LOG_PATH

from fkie_mas_pylib import formats

from .sensor_interface import SensorInterface


class HddUsage(SensorInterface):
    def __init__(self, hostname="", interval=30.0, warn_level=0.95, window=10.0):
        self._hdd_usage_warn = warn_level
        self._path = LOG_PATH
        SensorInterface.__init__(self, hostname, sensorname="HDD Usage", interval=interval, window=window)

    def reload_parameter(self, settings):
        self._hdd_usage_warn = settings.param("sysmon/Disk/usage_warn_level", self._hdd_usage_warn)
        new_path = settings.param("sysmon/Disk/path", self._path)
        if new_path != self._path:
            # values of the previous path must not influence the average
            self.clear_samples()
        self._path = new_path
        # averaging window, clamped to the measurement interval by the setter
        self.window = settings.param("sysmon/Disk/window", self.window)

    def check_sensor(self):
        diag_level = DiagnosticStatus.OK
        diag_vals = []
        diag_msg = ""
        try:
            hdd = psutil.disk_usage(self._path)
            if hdd.total <= 0:
                # no usable disk information, avoid division by zero
                return
            warn_on_space = hdd.total * (1.0 - self._hdd_usage_warn)
            diag_msg = "warn at >%s%% (<%s, avg over %.0fs)" % (
                self._hdd_usage_warn * 100.0,
                formats.sizeof_fmt(warn_on_space),
                self.window,
            )
            # relax the threshold while a warning is already active
            warn_level = self.hysteresis(warn_on_space, factor=1.1)
            # add current measurement to the sliding window
            self.add_sample({"free": float(hdd.free), "free_percent": 100.0 - hdd.percent})
            stats = self.window_stats()
            free_avg = stats["free"]["avg"]
            free_percent_avg = stats["free_percent"]["avg"]
            window_span = self.window_span()
            if free_avg <= warn_level:
                diag_level = DiagnosticStatus.WARN
                diag_msg = "Free disk space on log path only %s (warn on <%s, avg over %.0fs)" % (
                    formats.sizeof_fmt(free_avg),
                    formats.sizeof_fmt(warn_on_space),
                    window_span,
                )
            diag_vals.append(KeyValue(key="Free", value="%d" % int(free_avg)))
            diag_vals.append(KeyValue(key="Free [%]", value="%.2f" % free_percent_avg))
            diag_vals.append(KeyValue(key="Window [s]", value="%.1f" % window_span))
            diag_vals.append(KeyValue(key="Path", value=self._path))
        except Exception as err:
            # e.g. path does not exist or is not accessible
            diag_level = DiagnosticStatus.WARN
            diag_msg = "%s" % err
            diag_vals = []
            diag_vals.append(KeyValue(key="Free", value=""))
            diag_vals.append(KeyValue(key="Free [%]", value=""))
            diag_vals.append(KeyValue(key="Path", value=self._path))

        # Update status
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg
