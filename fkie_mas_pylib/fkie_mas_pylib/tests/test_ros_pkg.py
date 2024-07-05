# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import unittest
import rospkg

from fkie_mas_pylib import ros_pkg
from fkie_mas_pylib.launch import xml

PKG = 'fkie_mas_pylib'


class TestRosPkgLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.pkg_path = xml.interpret_path("$(find %s)/" % PKG)

    def tearDown(self):
        pass

    def test_get_packages(self):
        path = os.path.dirname(self.pkg_path.rstrip(os.path.sep))
        pkg_res = ros_pkg.get_packages(path)
        count_exp = 7
        if 'industrial_ci' in pkg_res:
            count_exp += 1
        self.assertEqual(count_exp, len(
            pkg_res), "wrong count of get_packages(%s), expected: %d, got: %d -> packages: %s" % (path, count_exp, len(pkg_res), pkg_res))

    def test_get_cwd(self):
        test_path = '/this/is/path/to'
        result_path = ros_pkg.get_cwd('node', '%s/bin' % test_path)
        self.assertEqual(test_path, result_path, "wrong get_cwd from 'node' type, expected: %s, got: %s" % (
            test_path, result_path))
        test_path = os.getcwd()
        result_path = ros_pkg.get_cwd('cwd', '')
        self.assertEqual(test_path, result_path, "wrong get_cwd from 'cwd' type, expected: %s, got: %s" % (
            test_path, result_path))
        test_path = rospkg.get_ros_root()
        result_path = ros_pkg.get_cwd('ros-root', '')
        self.assertEqual(test_path, result_path,
                         "wrong get_cwd from 'ros-root' type, expected: %s, got: %s" % (test_path, result_path))
        test_path = rospkg.get_ros_home()
        result_path = ros_pkg.get_cwd('', '')
        self.assertEqual(test_path, result_path, "wrong get_cwd from empty type, expected: %s, got: %s" % (
            test_path, result_path))

    def test_package_name(self):
        pkg = ros_pkg.get_name(self.pkg_path)
        self.assertEqual(PKG, pkg[0], "wrong package name, expected: %s, got: %s" % (
            PKG, pkg[0]))
        pkg_dir = self.pkg_path
        self.assertEqual(
            pkg_dir, pkg[1], "wrong package path, expected: %s, got: %s" % (pkg_dir, pkg[1]))
        # test cache
        pkg = ros_pkg.get_name(self.pkg_path)
        self.assertEqual(PKG, pkg[0], "wrong package name from cache, expected: %s, got: %s" % (
            PKG, pkg[0]))
        self.assertEqual(
            pkg_dir, pkg[1], "wrong package path from cache, expected: %s, got: %s" % (pkg_dir, pkg[1]))
        # test invalid path
        pkg = ros_pkg.get_name('INVALID')
        self.assertEqual(
            '', pkg[0], "wrong package name, expected: %s, got: %s" % (None, pkg[0]))
        self.assertEqual(
            '', pkg[1], "wrong package name, expected: %s, got: %s" % (None, pkg[1]))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestRosPkgLib)
