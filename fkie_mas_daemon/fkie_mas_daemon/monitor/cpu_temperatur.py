# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import psutil
import rclpy
import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from .sensor_interface import SensorInterface
import fkie_mas_daemon as nmd


class CpuTemperatur(SensorInterface):

    def __init__(self, hostname='', interval=5.0, warn_level=85.0):
        # self._cpu_temp_warn = rospy.get_param('~cpu_temp_warn', warn_level)
        self._cpu_temp_warn = warn_level
        SensorInterface.__init__(
            self, hostname, sensorname='CPU Temperature', interval=interval)

    def reload_parameter(self, settings):
        pass

    def check_sensor(self):
        try:
            sensor_temps = psutil.sensors_temperatures()
            diag_level = DiagnosticStatus.OK
            diag_vals = []
            diag_msg = 'warn at >%.2f&deg;C' % (self._cpu_temp_warn)
            warn_level = self._cpu_temp_warn
            if diag_level == DiagnosticStatus.WARN:
                warn_level = warn_level * 0.9
            max_temp = 0
            for sensor, shwtemps in sensor_temps.items():
                if sensor == 'coretemp':
                    for _label, current, hight, _critical in shwtemps:
                        if hight is not None:
                            self._cpu_temp_warn = hight
                        if current > max_temp:
                            max_temp = current
            if max_temp > warn_level:
                diag_msg = 'CPU Temperature: %.2f degree (warn level >%.2f)' % (
                    max_temp, self._cpu_temp_warn)
            diag_vals.append(
                KeyValue(key='Max [degree]', value='%.2f' % max_temp))
            # Update status
            with self.mutex:
                self._ts_last = time.time()
                self._stat_msg.level = diag_level
                self._stat_msg.values = diag_vals
                self._stat_msg.message = diag_msg
        except Exception as error:
            import traceback
            print(traceback.format_exc())
            nmd.ros_node.get_logger().warn(
                "Sensor temperatures are not checked because of error: %s" % error)
            self._interval = 0
