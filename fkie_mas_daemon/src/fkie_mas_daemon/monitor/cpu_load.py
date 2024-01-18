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

import psutil
import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from .sensor_interface import SensorInterface


class CpuLoad(SensorInterface):

    def __init__(self, hostname='', interval=5.0, warn_level=0.9):
        self._cpu_load_warn = warn_level
        self._count_processes = 3
        SensorInterface.__init__(
            self, hostname, sensorname='CPU Load', interval=interval)

    def reload_parameter(self, settings):
        self._cpu_load_warn = settings.param(
            'sysmon/CPU/load_warn_level', self._cpu_load_warn)
        self._count_processes = settings.param('sysmon/CPU/count_processes', 3)

    def check_sensor(self):
        cpu_percents = psutil.cpu_percent(interval=None, percpu=True)
        diag_level = 0
        diag_vals = []
        diag_msg = 'warn at >%.2f%%' % (self._cpu_load_warn * 100.0)
        warn_level = self._cpu_load_warn
        if diag_level == DiagnosticStatus.WARN:
            warn_level = warn_level * 0.9
        count_warn_cpu = 0
        cpu_max_percent = 0
        cpu_percent_total = 0
        for cpu_idx in range(len(cpu_percents)):
            cpu_percent = cpu_percents[cpu_idx]
            if cpu_percent > cpu_max_percent:
                cpu_max_percent = cpu_percent
            if cpu_percents[cpu_idx] / 100.0 >= warn_level:
                count_warn_cpu += 1
            cpu_percent_total += cpu_percent
        diag_vals.append(KeyValue(key='Max [%]', value=cpu_max_percent))
        diag_vals.append(KeyValue(key='Avg [%]', value='%.2f' % (
            cpu_percent_total / len(cpu_percents))))
        if count_warn_cpu > 1:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'CPU load of %d cores is >%.0f%%' % (
                count_warn_cpu, self._cpu_load_warn * 100)
            try:
                # determine processes with high load
                processes = []
                for pi in sorted(psutil.process_iter(attrs=['name', 'cpu_percent']), key=lambda pi: pi.info['cpu_percent'], reverse=True):
                    if pi.info['cpu_percent'] / 100.0 >= warn_level:
                        phlmsg = '%.2f%% %s [%d]' % (
                            pi.info['cpu_percent'], pi.info['name'], pi.pid)
                        processes.append(phlmsg)
                    if len(processes) >= self._count_processes:
                        break
                for msg in processes:
                    diag_vals.append(KeyValue(key='Process load', value=msg))
            except Exception:
                pass
        # Update status
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg
