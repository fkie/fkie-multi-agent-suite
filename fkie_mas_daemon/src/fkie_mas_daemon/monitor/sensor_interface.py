# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import rospy
import threading

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from fkie_mas_pylib import formats


class SensorInterface(threading.Thread):

    def __init__(self, hostname='', sensorname='noname', interval=1.0):
        threading.Thread.__init__(self, daemon=True)
        self.hostname = hostname
        self.mutex = threading.RLock()
        self._interval = interval
        self._stat_msg = DiagnosticStatus()
        self._stat_msg.name = '%s' % (sensorname)
        self._stat_msg.level = 3
        self._stat_msg.hardware_id = hostname
        self._stat_msg.message = 'No Data'
        self._stat_msg.values = []
        self._ts_last = 0
        self._stop_event = threading.Event()
        self.start()

    # @abc.abstractmethod
    def check_sensor(self):
        pass

    # @abc.abstractmethod
    def reload_parameter(self, settings):
        pass

    def run(self):
        while not self._stop_event.wait(self._interval):
            self.check_sensor()

    def stop(self):
        self._stop_event.set()

    def last_state(self, ts_now, filter_level=0, filter_ts=0):
        '''
        :param float ts_now: current timestamp
        :param int filter_level: minimal level
        :param float filter_ts: only message after this timestamp
        :return: last state if data is available. In other case it should be None
        :rtype: diagnostic_msgs.msg.DiagnosticStatus
        '''
        with self.mutex:
            if self._ts_last > 0:
                if self._ts_last > filter_ts and self._stat_msg.level >= filter_level:
                    self.update_value_last_ts(
                        self._stat_msg, ts_now, self._ts_last)
                    return self._stat_msg
        return None

    def update_value_last_ts(self, msg, nowts, ts):
        if msg.values and msg.values[-1].key == 'Timestamp':
            del msg.values[-1]
        msg.values.append(
            KeyValue(key='Timestamp', value=formats.timestamp_fmt(ts, False, False)))
