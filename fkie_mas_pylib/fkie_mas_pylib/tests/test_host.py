# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import unittest
import time

import fkie_mas_pylib.system.host as host

PKG = 'fkie_mas_pylib'


class TestHost(unittest.TestCase):
    '''
    '''

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_get_hostname(self):
        hostname = host.get_hostname(None)
        self.assertEqual(
            hostname, None, "Hostname from `None` should be `None`, got: %s, expected: %s" % (hostname, None))
        hostname = host.get_hostname('')
        self.assertEqual(
            hostname, '', "Hostname from `` should be ``, got: %s, expected: ''" % (hostname))
        name = 'myhost'
        hostname = host.get_hostname('http://%s' % name)
        self.assertEqual(
            hostname, name, "Wrong hostname from `http://%s`, got: %s, expected: %s" % (name, hostname, name))
        hostname = host.get_hostname('http://%s:11111' % name)
        self.assertEqual(
            hostname, name, "Wrong hostname from `http://%s:11111`, got: %s, expected: %s" % (name, hostname, name))
        hostname = host.get_hostname('%s:11111' % name)
        self.assertEqual(hostname, name, "Wrong hostname from `%s:11111`, got: %s, expected: %s" % (
            name, hostname, name))
        wrong = '%s:11:2' % name
        hostname = host.get_hostname(wrong)
        self.assertEqual(hostname, wrong, "Wrong hostname from `%s`, got: %s, expected: %s" % (
            wrong, hostname, wrong))

    def test_get_ros_hostname(self):
        roshn = host.get_ros_hostname(None)
        self.assertEqual(
            roshn, '', "ros hostname from `None` should be ``, got: %s, expected: ''" % (roshn))
        roshn = host.get_ros_hostname('http://myhost:11311')
        self.assertEqual(
            roshn, 'myhost', "wrong ros hostname from `'http://myhost:11311'`, got: %s, expected: %s" % (roshn, 'myhost'))
        roshn = host.get_ros_hostname('http://192.168.11.5:11311')
        self.assertEqual(
            roshn, '', "wrong ros hostname from `'http://192.168.11.5:11311'`, got: %s, expected: ''" % (roshn))

    def test_is_local(self):
        local = host.is_local(None)
        self.assertEqual(
            local, True, "wrong is_local from `None`, got: %s, expected: %s" % (local, True))
        local = host.is_local('localhost')
        self.assertEqual(
            local, True, "wrong is_local from `localhost`, got: %s, expected: %s" % (local, True))
        local = host.is_local('localhost')
        self.assertEqual(
            local, True, "wrong cached is_local from `localhost`, got: %s, expected: %s" % (local, True))
        local = host.is_local('127.0.0.1')
        self.assertEqual(
            local, True, "wrong cached is_local from `127.0.0.1`, got: %s, expected: %s" % (local, True))
        local = host.is_local('heise.de', False)
        self.assertEqual(
            local, False, "wrong is_local from `heise.de`, got: %s, expected: %s" % (local, False))
        local = host.is_local('heise.de', False)
        self.assertEqual(
            local, False, "wrong cached is_local from `heise.de`, got: %s, expected: %s" % (local, False))
        local = host.is_local('unknown', False)
        self.assertEqual(
            local, False, "wrong is_local from `unknown`, got: %s, expected: %s" % (local, False))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestHost)
