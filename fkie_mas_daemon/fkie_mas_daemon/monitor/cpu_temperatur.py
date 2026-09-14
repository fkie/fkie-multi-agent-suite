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
import fkie_mas_daemon as nmd


class CpuTemperatur(SensorInterface):

    # sensor names which are known to report cpu package or core temperatures
    CPU_SENSOR_NAMES = ('coretemp', 'k10temp', 'zenpower', 'cpu_thermal',
                        'cpu-thermal', 'soc_thermal', 'acpitz')

    def __init__(self, hostname: str = '', interval: float = 5.0, warn_level: float = 85.0,
                 window: float = 10.0, count_processes: int = 3):
        self._cpu_temp_warn = warn_level
        # the warn level may be replaced by the value reported by the hardware
        # ('high'), but only as long as no parameter was configured
        self._warn_level_configured = False
        self._count_processes = count_processes
        SensorInterface.__init__(
            self, hostname, sensorname='CPU Temperature', interval=interval, window=window)

    def reload_parameter(self, settings):
        warn_level = settings.param(
            'sysmon/CPU/temperature_warn_level', self._cpu_temp_warn)
        if warn_level != self._cpu_temp_warn:
            # an explicit parameter has priority over the hardware value
            self._cpu_temp_warn = warn_level
            self._warn_level_configured = True
        self._count_processes = settings.param(
            'sysmon/CPU/count_processes', self._count_processes)
        # averaging window, clamped to the measurement interval by the setter
        self.window = settings.param('sysmon/CPU/window', self.window)

    def _disable(self, message: str):
        '''
        Stops the periodic measurement, e.g. if no temperature sensor exists.
        '''
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = DiagnosticStatus.OK
            self._stat_msg.values = []
            self._stat_msg.message = message
        self._interval = 0
        self.cancel_timer()

    def check_sensor(self):
        try:
            sensor_temps = psutil.sensors_temperatures()
        except Exception as error:
            # sensors_temperatures() is not available on all platforms
            nmd.ros_node.get_logger().warn(
                "Sensor temperatures are not checked because of error: %s" % error)
            self._disable('no temperature sensors available')
            return
        max_temp = None
        max_label = ''
        temp_high = None
        temp_critical = None
        for sensor, shwtemps in sensor_temps.items():
            if sensor not in self.CPU_SENSOR_NAMES:
                continue
            for entry in shwtemps:
                # use attribute access, the namedtuple layout differs between
                # psutil versions
                current = getattr(entry, 'current', None)
                if current is None:
                    continue
                if max_temp is None or current > max_temp:
                    max_temp = current
                    max_label = getattr(entry, 'label', '') or sensor
                    temp_high = getattr(entry, 'high', None)
                    temp_critical = getattr(entry, 'critical', None)
        if max_temp is None:
            # no cpu sensor found, do not report a fake temperature of 0
            nmd.ros_node.get_logger().info(
                "CpuTemperatur: no CPU temperature sensor found, sensor disabled")
            self._disable('no CPU temperature sensor found')
            return
        now = time.time()
        # use the threshold of the hardware if no parameter was configured
        if not self._warn_level_configured and temp_high is not None and temp_high > 0:
            self._cpu_temp_warn = temp_high
        diag_level = DiagnosticStatus.OK
        diag_vals = []
        diag_msg = 'warn at >%.2f&deg;C (avg over %.0fs)' % (
            self._cpu_temp_warn, self.window)
        # relax the threshold while a warning is already active
        warn_level = self.hysteresis(self._cpu_temp_warn, factor=0.9)
        # add current measurement to the sliding window
        self.add_sample({'temperature': float(max_temp)}, ts=now)
        stats = self.window_stats()
        temp_avg = stats['temperature']['avg']
        temp_max = stats['temperature']['max']
        window_span = self.window_span()
        diag_vals.append(
            KeyValue(key='Max [degree]', value='%.2f' % temp_avg))
        diag_vals.append(
            KeyValue(key='Peak [degree]', value='%.2f' % temp_max))
        diag_vals.append(KeyValue(key='Sensor', value=max_label))
        diag_vals.append(
            KeyValue(key='Window [s]', value='%.1f' % window_span))
        if temp_critical is not None and temp_avg >= temp_critical:
            # above the critical value reported by the hardware
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'CPU Temperature: %.2f degree (critical >=%.2f)' % (
                temp_avg, temp_critical)
        elif temp_avg > warn_level:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'CPU Temperature: %.2f degree (warn level >%.2f, avg over %.0fs)' % (
                temp_avg, self._cpu_temp_warn, window_span)
        if diag_level != DiagnosticStatus.OK:
            # report the processes which most likely cause the temperature,
            # the values are shared with the CPU load sensor
            for msg in format_process_load(min_percent=1.0, count=self._count_processes,
                                           normalized=True, max_age=self._interval):
                diag_vals.append(KeyValue(key='Process load', value=msg))
        # Update status
        with self.mutex:
            self._ts_last = now
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg
