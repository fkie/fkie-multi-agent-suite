# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import abc
import os
import rclpy
import threading
import time

from collections import deque
from typing import Callable, Dict, Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from fkie_mas_pylib import formats
from fkie_mas_pylib.settings import Settings
import fkie_mas_daemon as nmd


_TRUE_VALUES = {"1", "true", "yes", "y", "on"}
_FALSE_VALUES = {"0", "false", "no", "n", "off", ""}


def env_bool(name: str, default: bool = False) -> bool:
    '''
    Reads an environment variable as boolean, robust against different spellings.

    :param str name: name of the environment variable
    :param bool default: result if the variable is not set or not parsable
    :rtype: bool
    '''
    raw = os.environ.get(name)
    if raw is None:
        return default
    value = raw.strip().lower()
    if value in _TRUE_VALUES:
        return True
    if value in _FALSE_VALUES:
        return False
    return default


class SensorInterface(object):
    __metaclass__ = abc.ABCMeta

    # hard limit for the sample count, protects against very short intervals
    MAX_SAMPLES = 1000

    def __init__(self, hostname: str = '', sensorname: str = 'noname', interval: float = 1.0, window: float = 10.0):
        '''
        :param str hostname: host the sensor belongs to
        :param str sensorname: name reported in the diagnostic message
        :param float interval: time in seconds between two measurements
        :param float window: length of the sliding window in seconds used to
                             average the measured values before reporting
        '''
        nmd.ros_node.get_logger().info("Loading monitor service: %s" % type(self).__name__)
        self.hostname = hostname
        self.mutex = threading.RLock()
        self._interval = interval
        # the averaging window can never be shorter than the measurement interval
        self._window = max(float(window), float(interval))
        # sliding window with (timestamp, {key: value}) samples
        self._samples = deque()
        self._timer = None
        self._stopped = False
        self._stat_msg = DiagnosticStatus()
        self._stat_msg.name = '%s' % sensorname
        self._stat_msg.level = DiagnosticStatus.STALE
        self._stat_msg.hardware_id = hostname
        self._stat_msg.message = 'No Data'
        self._stat_msg.values = []
        self._ts_last = 0
        # MAS_SYSTEM_DIAGNOSTIC=1/true enables the diagnostics (default)
        self._disabled_by_env = not env_bool("MAS_SYSTEM_DIAGNOSTIC", True)
        self._start_check_sensor()

    @property
    def window(self) -> float:
        '''
        :return: length of the averaging window in seconds
        :rtype: float
        '''
        with self.mutex:
            return self._window

    @window.setter
    def window(self, value: float):
        '''
        Sets the length of the averaging window. Values shorter than the
        measurement interval are clamped to the measurement interval.
        '''
        with self.mutex:
            self._window = max(float(value), float(self._interval))

    @staticmethod
    def level_as_int(level) -> int:
        '''
        Converts a diagnostic level to int. Depending on the rclpy version the
        level is either an int or a single byte.

        :rtype: int
        '''
        if isinstance(level, int):
            return level
        if isinstance(level, (bytes, bytearray)):
            return int.from_bytes(level, byteorder='big')
        return int(level)

    def hysteresis(self, threshold: float, factor: float = 0.9) -> float:
        '''
        Applies a factor to the given threshold while the last reported state
        was a warning or error. This avoids flapping between OK and WARN.

        :param float threshold: the configured threshold
        :param float factor: factor applied while a warning is active
        :rtype: float
        '''
        with self.mutex:
            last_level = self.level_as_int(self._stat_msg.level)
        # STALE must not be treated as warning, it is the initial level
        if last_level in (DiagnosticStatus.WARN, DiagnosticStatus.ERROR):
            return threshold * factor
        return threshold

    def add_sample(self, values: Dict[str, float], ts: Optional[float] = None) -> float:
        '''
        Adds a measurement to the sliding window and drops outdated samples.

        :param dict values: measured values, e.g. {'cpu0': 12.5}
        :param float ts: timestamp of the measurement, current time if None
        :return: the timestamp used for the sample
        :rtype: float
        '''
        if ts is None:
            ts = time.time()
        with self.mutex:
            self._samples.append((ts, dict(values)))
            min_ts = ts - self._window
            # keep at least one sample to always be able to report a value
            while len(self._samples) > 1 and self._samples[0][0] < min_ts:
                self._samples.popleft()
            while len(self._samples) > self.MAX_SAMPLES:
                self._samples.popleft()
            return ts

    def window_stats(self) -> Dict[str, Dict[str, float]]:
        '''
        Aggregates all samples of the current window.

        :return: {key: {'avg': float, 'min': float, 'max': float, 'last': float, 'count': int}}
        :rtype: dict
        '''
        result = {}
        with self.mutex:
            samples = list(self._samples)
        for _ts, values in samples:
            for key, value in values.items():
                entry = result.get(key)
                if entry is None:
                    result[key] = {'sum': float(value), 'count': 1, 'min': float(value),
                                   'max': float(value), 'last': float(value)}
                else:
                    entry['sum'] += float(value)
                    entry['count'] += 1
                    entry['min'] = min(entry['min'], float(value))
                    entry['max'] = max(entry['max'], float(value))
                    entry['last'] = float(value)
        for entry in result.values():
            # average over all samples inside the window
            entry['avg'] = entry['sum'] / entry['count']
            del entry['sum']
        return result

    def window_span(self) -> float:
        '''
        :return: time span in seconds covered by the current samples
        :rtype: float
        '''
        with self.mutex:
            if len(self._samples) < 2:
                return 0.0
            return self._samples[-1][0] - self._samples[0][0]

    def clear_samples(self):
        '''
        Removes all samples of the sliding window.
        '''
        with self.mutex:
            self._samples.clear()

    @abc.abstractmethod
    def check_sensor(self):
        pass

    @abc.abstractmethod
    def reload_parameter(self, settings: Settings):
        pass

    def _start_check_sensor(self):
        if not self.is_active():
            return
        ts_start = time.monotonic()
        try:
            self.check_sensor()
        except Exception as err:
            # an exception must not break the timer chain
            nmd.ros_node.get_logger().debug("%s: error while check_sensor(): %s" % (
                type(self).__name__, err))
        if self.is_active() and self._interval > 0:
            # compensate the duration of the measurement to avoid drift
            elapsed = time.monotonic() - ts_start
            self.start_timer(max(0.0, self._interval - elapsed),
                             self._start_check_sensor)

    def last_state(self, ts_now: float = 0, filter_level: int = 0, filter_ts: float = 0):
        '''
        :param float ts_now: current timestamp
        :param int filter_level: minimal level
        :param float filter_ts: only message after this timestamp
        :return: last state if data is available. In other case it should be None
        :rtype: diagnostic_msgs.msg.DiagnosticStatus
        '''
        with self.mutex:
            if self._ts_last > 0:
                if self._ts_last > filter_ts and self.level_as_int(self._stat_msg.level) >= filter_level:
                    self.update_value_last_ts(
                        self._stat_msg, ts_now, self._ts_last)
                    return self._stat_msg
        return None

    def update_value_last_ts(self, msg, nowts: float = 0, ts: float = 0):
        '''
        Replaces the 'Timestamp' entry of the message by the given timestamp.

        :param float nowts: current timestamp, kept for backward compatibility
        :param float ts: timestamp of the last measurement
        '''
        if msg.values and msg.values[-1].key == 'Timestamp':
            del msg.values[-1]
        msg.values.append(
            KeyValue(key='Timestamp', value=formats.timestamp_fmt(ts, False, False)))

    def is_active(self):
        if self._disabled_by_env:
            return False
        if not rclpy.ok():
            with self.mutex:
                self.cancel_timer()
            return False
        return True

    def start_timer(self, interval: float, callback: Callable):
        with self.mutex:
            if self._stopped:
                return
            self._timer = threading.Timer(interval, callback)
            self._timer.daemon = True
            self._timer.start()

    def cancel_timer(self):
        with self.mutex:
            self._stopped = True
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
