
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
