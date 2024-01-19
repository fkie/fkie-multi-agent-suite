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
import time
import rospkg

from fkie_mas_pylib.system import ros1_masteruri
from fkie_mas_pylib.system import url

PKG = 'fkie_mas_daemon'


class TestUrlLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.test_include_file = "%s/resources/include_dummy.launch" % os.getcwd()
        self.master_port = url.get_port(ros1_masteruri.from_master(True))

    def tearDown(self):
        pass

    def test_get_port(self):
        port = url.get_port(None)
        self.assertEqual(
            port, None, "Port from `None` should be `None`, got: %s, expected: %s" % (port, None))
        port = url.get_port('')
        self.assertEqual(
            port, '', "Port from `` should be ``, got: %s, expected: ''" % (port))
        port = url.get_port('host:21')
        self.assertEqual(
            port, 21, "wrong port from `:21`, got: %s, expected: %d" % (port, 21))
        port = url.get_port('https://host:21')
        self.assertEqual(
            port, 21, "wrong port from `https://host:21`, got: %s, expected: %d" % (port, 21))
        port = url.get_port('https://host:s21')
        self.assertEqual(
            port, None, "wrong port from `https://host:s21`, got: %s, expected: %s" % (port, None))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestUrlLib)
