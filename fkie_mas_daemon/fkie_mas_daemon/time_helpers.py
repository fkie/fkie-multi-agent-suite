# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from rclpy.time import Time, CONVERSION_CONSTANT


def rostime2float(rcltime: Time):
    return float('.'.join(str(ele) for ele in rcltime.seconds_nanoseconds()))


def float2rostime(value: float):
    seconds = int(value)
    nanoseconds = (value - seconds) * CONVERSION_CONSTANT
    return Time(seconds=seconds, nanoseconds=nanoseconds)
