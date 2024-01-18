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

import os
import unittest
from datetime import tzinfo, timedelta

from fkie_mas_pylib import formats

PKG = 'fkie_mas_pylib'


class TestFormatsLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_sizeof_fmt(self):
        sizeof_str = formats.sizeof_fmt(12345678)
        sizeof_str_res = "12MiB"
        self.assertEqual(sizeof_str_res, sizeof_str, "wrong sizeof_fmt, expected: %s, got: '%s'" % (
            sizeof_str_res, sizeof_str))

    def test_formated_ts(self):

        class UTC(tzinfo):
            def utcoffset(self, dt):
                return timedelta(0)

            def tzname(self, dt):
                return "UTC"

            def dst(self, dt):
                return timedelta(0)

        tsstr_ff = formats.timestamp_fmt(
            1557480759.608808, with_date=False, with_nanosecs=False, tz=UTC())
        tsstr_ff_res = "09:32:39"
        self.assertEqual(tsstr_ff_res, tsstr_ff, "wrong timestamp_fmt(value, False, False), expected: %s, got: %s" % (
            tsstr_ff_res, tsstr_ff))
        tsstr_tf = formats.timestamp_fmt(
            1557480759.608808, with_date=True, with_nanosecs=False, tz=UTC())
        tsstr_tf_res = "09:32:39 (10.05.2019)"
        self.assertEqual(tsstr_tf_res, tsstr_tf, "wrong timestamp_fmt(value, True, False), expected: %s, got: %s" % (
            tsstr_tf_res, tsstr_tf))
        tsstr_tt = formats.timestamp_fmt(
            1557480759.608808, with_date=True, with_nanosecs=True, tz=UTC())
        tsstr_tt_res = "09:32:39.608808 (10.05.2019)"
        self.assertEqual(tsstr_tt_res, tsstr_tt, "wrong timestamp_fmt(value, True, True), expected: %s, got: %s" % (
            tsstr_tt_res, tsstr_tt))
        tsstr_ft = formats.timestamp_fmt(
            1557480759.608808, with_date=False, with_nanosecs=True, tz=UTC())
        tsstr_ft_res = "09:32:39.608808"
        self.assertEqual(tsstr_ft_res, tsstr_ft, "wrong timestamp_fmt(value, False, True), expected: %s, got: %s" % (
            tsstr_ft_res, tsstr_ft))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestFormatsLib)
