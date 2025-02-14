# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import unittest

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
