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
from fkie_mas_pylib.defines import LOG_PATH
from .sensor_interface import SensorInterface


class HddUsage(SensorInterface):

    def __init__(self, hostname='', interval=30.0, warn_level=0.95):
        self._hdd_usage_warn = warn_level
        self._path = LOG_PATH
        SensorInterface.__init__(
            self, hostname, sensorname='HDD Usage', interval=interval)

    def reload_parameter(self, settings):
        self._hdd_usage_warn = settings.param(
            'sysmon/Disk/usage_warn_level', self._hdd_usage_warn)
        self._path = settings.param('sysmon/Disk/path', self._path)

    def check_sensor(self):
        diag_level = DiagnosticStatus.OK
        diag_vals = []
        diag_msg = ''
        try:
            hdd = psutil.disk_usage(self._path)
            diag_level = DiagnosticStatus.OK
            diag_vals = []
            warn_on_space = hdd.total * (1.0 - self._hdd_usage_warn)
            diag_msg = 'warn at >%s%% (<%s)' % (
                self._hdd_usage_warn * 100., formats.sizeof_fmt(warn_on_space))
            warn_level = warn_on_space
            if diag_level == DiagnosticStatus.WARN:
                warn_level = warn_level * 1.1
            if hdd.free <= warn_on_space:
                diag_level = DiagnosticStatus.WARN
                diag_msg = 'Free disk space on log path only %s (warn on <%s)' % (
                    formats.sizeof_fmt(hdd.free), formats.sizeof_fmt(warn_on_space))
            # print "Percent Disk %.2f" % (hdd.percent), diag_level
            diag_vals.append(KeyValue(key='Free', value='%d' % hdd.free))
            diag_vals.append(
                KeyValue(key='Free [%]', value='%.2f' % (100.0 - hdd.percent)))
            diag_vals.append(KeyValue(key='Path', value=self._path))
        except Exception as err:
            warn_level = DiagnosticStatus.WARN
            diag_msg = '%s' % err
            diag_vals.append(KeyValue(key='Free', value=""))
            diag_vals.append(KeyValue(key='Free [%]', value=""))
            diag_vals.append(KeyValue(key='Path', value=self._path))

        # Update status
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg
