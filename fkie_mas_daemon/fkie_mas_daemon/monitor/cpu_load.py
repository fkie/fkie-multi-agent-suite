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
from .process_load import format_process_load
from .sensor_interface import SensorInterface


class CpuLoad(SensorInterface):

    def __init__(self, hostname: str = '', interval: float = 5.0, warn_level: float = 0.9, window: float = 10.0):
        self._cpu_load_warn = warn_level
        self._count_processes = 3
        # the first psutil measurement always returns 0.0
        self._first_measurement = True
        SensorInterface.__init__(
            self, hostname, sensorname='CPU Load', interval=interval, window=window)

    def reload_parameter(self, settings):
        self._cpu_load_warn = settings.param(
            'sysmon/CPU/load_warn_level', self._cpu_load_warn)
        self._count_processes = settings.param('sysmon/CPU/count_processes', 3)
        # averaging window, clamped to the measurement interval by the setter
        self.window = settings.param('sysmon/CPU/window', self.window)

    def check_sensor(self):
        if self._first_measurement:
            # a short blocking call is needed for the very first measurement,
            # otherwise psutil returns 0.0 for all cores
            cpu_percents = psutil.cpu_percent(interval=0.3, percpu=True)
            self._first_measurement = False
        else:
            cpu_percents = psutil.cpu_percent(interval=None, percpu=True)
        if not cpu_percents:
            # no cpu information available, avoid division by zero
            return
        now = time.time()
        diag_level = DiagnosticStatus.OK
        diag_vals = []
        diag_msg = 'warn at >%.2f%% (avg over %.0fs)' % (
            self._cpu_load_warn * 100.0, self.window)
        # relax the threshold while a warning is already active
        warn_level = self.hysteresis(self._cpu_load_warn, factor=0.9)
        # add current measurement to the sliding window
        self.add_sample({'cpu%d' % cpu_idx: cpu_percent for cpu_idx,
                         cpu_percent in enumerate(cpu_percents)}, ts=now)
        stats = self.window_stats()
        # average load per core over the whole window
        core_avgs = [entry['avg'] for entry in stats.values()]
        cpu_max_percent = max(core_avgs)
        cpu_avg_percent = sum(core_avgs) / len(core_avgs)
        count_warn_cpu = len(
            [value for value in core_avgs if value / 100.0 >= warn_level])
        window_span = self.window_span()
        diag_vals.append(
            KeyValue(key='Max [%]', value='%.2f' % cpu_max_percent))
        diag_vals.append(
            KeyValue(key='Avg [%]', value='%.2f' % cpu_avg_percent))
        diag_vals.append(
            KeyValue(key='Window [s]', value='%.1f' % window_span))
        if count_warn_cpu > 1:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'CPU load of %d cores is >%.0f%% (avg over %.0fs)' % (
                count_warn_cpu, self._cpu_load_warn * 100, window_span)
            # determine processes with high load, values are shared with the
            # CPU temperature sensor
            for msg in format_process_load(min_percent=warn_level * 100.0,
                                           count=self._count_processes,
                                           normalized=True, max_age=self._interval):
                diag_vals.append(KeyValue(key='Process load', value=msg))
        # Update status
        with self.mutex:
            self._ts_last = now
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg
