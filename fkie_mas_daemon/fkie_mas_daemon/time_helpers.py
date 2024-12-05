# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from rclpy.time import Time
try:
    from rclpy.constants import S_TO_NS
except:
    from rclpy.time import CONVERSION_CONSTANT as S_TO_NS


def rostime2float(rcltime: Time):
    return float('.'.join(str(ele) for ele in rcltime.seconds_nanoseconds()))


def float2rostime(value: float):
    seconds = int(value)
    nanoseconds = (value - seconds) * S_TO_NS
    return Time(seconds=seconds, nanoseconds=nanoseconds)
