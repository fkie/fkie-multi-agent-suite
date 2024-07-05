
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from typing import Text
from typing import Union

from datetime import datetime


def sizeof_fmt(num: Union[float, int], suffix: str = 'B') -> Text:
    for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
        if abs(num) < 1024.0:
            return '%.0f%s%s' % (num, unit, suffix)
        num /= 1024.0
    return '%.0f%s%s' % (num, 'YiB', suffix)


def timestamp_fmt(stamp: float, with_date: bool = True, with_nanosecs: bool = True, tz=None) -> Text:
    ts = stamp
    if hasattr(stamp, 'secs'):
        ts = stamp.secs + stamp.secs / 1000000000.
    str_format = '%H:%M:%S'
    if with_nanosecs:
        str_format += '.%f'
    if with_date:
        str_format += ' (%d.%m.%Y)'
    return datetime.fromtimestamp(ts, tz).strftime(str_format)
