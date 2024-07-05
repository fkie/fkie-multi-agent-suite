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

    def __init__(self, hostname='', interval=5.0, warn_level=0.95):
        self._mem_usage_warn = warn_level
        SensorInterface.__init__(
            self, hostname, sensorname='Memory Usage', interval=interval)

    def reload_parameter(self, settings):
        self._mem_usage_warn = settings.param(
            'sysmon/Memory/usage_warn_level', self._mem_usage_warn)

    def check_sensor(self):
        mem = psutil.virtual_memory()
        diag_level = 0
        diag_vals = []
        warn_on_mem = mem.total * (1.0 - self._mem_usage_warn)
        diag_msg = 'warn at >%s%% (<%s)' % (
            self._mem_usage_warn * 100., formats.sizeof_fmt(warn_on_mem))
        warn_level = warn_on_mem
        if diag_level == DiagnosticStatus.WARN:
            warn_level = warn_level * 1.1
        if mem.total - mem.used <= warn_on_mem:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'Memory available %s (warn <%s)' % (
                formats.sizeof_fmt(mem.total - mem.used), formats.sizeof_fmt(warn_on_mem))
        # print "MEM available ", mem.available, diag_level
        diag_vals.append(KeyValue(key='Free', value=mem.total - mem.used))
        diag_vals.append(KeyValue(key='Free [%]', value='%.2f' % (
            float(mem.total - mem.used) * 100.0 / float(mem.total))))

        # Update status
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg
