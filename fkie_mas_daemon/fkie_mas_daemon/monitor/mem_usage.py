# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import psutil
import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from fkie_mas_pylib import formats
from .sensor_interface import SensorInterface


class MemUsage(SensorInterface):

    def __init__(self, hostname='', interval=5.0, warn_level=0.95, window=10.0):
        self._mem_usage_warn = warn_level
        SensorInterface.__init__(
            self, hostname, sensorname='Memory Usage', interval=interval, window=window)

    def reload_parameter(self, settings):
        self._mem_usage_warn = settings.param(
            'sysmon/Memory/usage_warn_level', self._mem_usage_warn)
        # averaging window, clamped to the measurement interval by the setter
        self.window = settings.param('sysmon/Memory/window', self.window)

    def check_sensor(self):
        mem = psutil.virtual_memory()
        if mem.total <= 0:
            # no usable memory information, avoid division by zero
            return
        diag_level = DiagnosticStatus.OK
        diag_vals = []
        warn_on_mem = mem.total * (1.0 - self._mem_usage_warn)
        diag_msg = 'warn at >%s%% (<%s, avg over %.0fs)' % (
            self._mem_usage_warn * 100., formats.sizeof_fmt(warn_on_mem), self.window)
        # relax the threshold while a warning is already active
        warn_level = self.hysteresis(warn_on_mem, factor=1.1)
        mem_free = float(mem.total - mem.used)
        # add current measurement to the sliding window
        self.add_sample({'free': mem_free,
                         'free_percent': mem_free * 100.0 / float(mem.total)})
        stats = self.window_stats()
        free_avg = stats['free']['avg']
        free_percent_avg = stats['free_percent']['avg']
        window_span = self.window_span()
        if free_avg <= warn_level:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'Memory available %s (warn <%s, avg over %.0fs)' % (
                formats.sizeof_fmt(free_avg), formats.sizeof_fmt(warn_on_mem), window_span)
        diag_vals.append(KeyValue(key='Free', value='%d' % int(free_avg)))
        diag_vals.append(
            KeyValue(key='Free [%]', value='%.2f' % free_percent_avg))
        diag_vals.append(
            KeyValue(key='Window [s]', value='%.1f' % window_span))

        # Update status
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg
