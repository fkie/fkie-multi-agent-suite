# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import time

import psutil
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from .sensor_interface import SensorInterface


class NetLoad(SensorInterface):
    def __init__(self, hostname="", interval=3.0, warn_level=0.9, window=10.0):
        self._net_load_warn = warn_level
        self._net_speed = 6
        self._net_stat_last = {}  # {iface: (sent, recv)}
        # timestamp of the last counter reading, used for the rate calculation
        self._ts_stat_last = 0.0
        self._interface = ""
        self.settings = None
        SensorInterface.__init__(self, hostname, sensorname="Network Load", interval=interval, window=window)

    def reload_parameter(self, settings):
        self._net_load_warn = settings.param("sysmon/Network/load_warn_level", self._net_load_warn)
        self._net_speed = settings.param("sysmon/Network/speed", self._net_speed)
        # TODO: support more than one interface
        new_interface = settings.param("sysmon/Network/interface", "")
        if new_interface != self._interface:
            # values of the previous interface must not influence the average
            self.clear_samples()
        self._interface = new_interface
        # averaging window, clamped to the measurement interval by the setter
        self.window = settings.param("sysmon/Network/window", self.window)
        self.settings = settings

    def check_sensor(self):
        net_stats = psutil.net_if_stats()
        net = psutil.net_io_counters(pernic=True)
        now = time.time()
        # duration since the last counter reading
        duration = now - self._ts_stat_last if self._ts_stat_last > 0 else 0.0
        self._ts_stat_last = now
        diag_level = DiagnosticStatus.OK
        diag_vals = []
        diag_msg = "warn at >%.2f%% at %.0fMBit (avg over %.0fs)" % (
            self._net_load_warn * 100.0,
            self._net_speed,
            self.window,
        )
        # relax the threshold while a warning is already active
        warn_level = self.hysteresis(self._net_load_warn, factor=0.9)
        # maximum rate in bytes per second
        max_rate = self._net_speed * 1024 * 1024 / 8.0
        interfaces = []
        parsed_interfaces = []
        samples = {}
        for net_if, net_if_stats in net_stats.items():
            interfaces.append(net_if)
            do_parse = net_if in net
            if not self._interface:
                do_parse = do_parse and net_if_stats.isup and net_if_stats.speed > 0
            else:
                do_parse = do_parse and net_if == self._interface
            if not do_parse:
                continue
            if self.settings is not None and not self._interface:
                # TODO: support more than one interface
                self._interface = net_if
                self.settings.set_param("sysmon/Network/interface", net_if)
            parsed_interfaces.append(net_if)
            net_values = net[net_if]
            stat_last = self._net_stat_last.get(net_if)
            # store current overall stats for next rate calculation
            self._net_stat_last[net_if] = (net_values.bytes_sent, net_values.bytes_recv)
            if stat_last is None or duration <= 0:
                # no reference values available yet, skip this cycle
                continue
            # in psutil versions below 5.3.0 there is no 'nowrap' argument. We need to calculate current rate itself.
            # negative values (counter wrap or interface reset) are clamped to 0
            bytes_sent_1s = max(0.0, (net_values.bytes_sent - stat_last[0]) / duration)
            bytes_recv_1s = max(0.0, (net_values.bytes_recv - stat_last[1]) / duration)
            samples["%s: sent" % net_if] = bytes_sent_1s
            samples["%s: recv" % net_if] = bytes_recv_1s
        if self.settings is not None:
            self.settings.set_param("sysmon/Network/interface", interfaces, tag=":alt")
        if not samples:
            # first cycle, no rates available yet
            return
        # add current measurement to the sliding window
        self.add_sample(samples, ts=now)
        stats = self.window_stats()
        window_span = self.window_span()
        for net_if in parsed_interfaces:
            sent_avg = stats.get("%s: sent" % net_if, {}).get("avg", 0.0)
            recv_avg = stats.get("%s: recv" % net_if, {}).get("avg", 0.0)
            # values are averaged over the window, key kept for compatibility
            diag_vals.append(KeyValue(key="%s: sent [1s]" % net_if, value="%.2f" % sent_avg))
            diag_vals.append(KeyValue(key="%s: recv [1s]" % net_if, value="%.2f" % recv_avg))
            percent_sent = sent_avg / max_rate if max_rate > 0 else 0.0
            percent_recv = recv_avg / max_rate if max_rate > 0 else 0.0
            if percent_sent >= warn_level or percent_recv >= warn_level:
                diag_level = DiagnosticStatus.WARN
                if percent_sent >= warn_level and percent_recv >= warn_level:
                    diag_msg = "Net load for sent is %.0f%% and recv %.0f%% (warn >%.0f%% [%dMBit], avg over %.0fs)" % (
                        percent_sent * 100,
                        percent_recv * 100,
                        self._net_load_warn * 100,
                        self._net_speed,
                        window_span,
                    )
                elif percent_sent >= warn_level:
                    diag_msg = "Net load for sent is %.0f%% (warn >%.0f%% [%dMBit], avg over %.0fs)" % (
                        percent_sent * 100,
                        self._net_load_warn * 100,
                        self._net_speed,
                        window_span,
                    )
                else:
                    diag_msg = "Net load for recv is %.0f%% (warn >%.0f%% [%dMBit], avg over %.0fs)" % (
                        percent_recv * 100,
                        self._net_load_warn * 100,
                        self._net_speed,
                        window_span,
                    )
        diag_vals.append(KeyValue(key="Window [s]", value="%.1f" % window_span))
        # Update status
        with self.mutex:
            self._ts_last = now
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg
