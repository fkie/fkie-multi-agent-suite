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
        diag_level = DiagnosticStatus.OK
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
        diag_vals.append(KeyValue(key='Free', value='%d' %
                                  (mem.total - mem.used)))
        diag_vals.append(KeyValue(key='Free [%]', value='%.2f' % (
            float(mem.total - mem.used) * 100.0 / float(mem.total))))

        # Update status
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg
