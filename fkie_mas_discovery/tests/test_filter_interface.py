# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import unittest

from fkie_mas_discovery.filter_interface import FilterInterface

PKG = 'fkie_mas_discovery'


class TestFilterInterface(unittest.TestCase):
    '''
    '''

    def setUp(self):
        pass

    def test_do_not_sync(self):
        fi = FilterInterface()

        fi.load(mastername='testmaster',
                ignore_nodes=[], sync_nodes=['/node_one', '/node_two/topic'],
                ignore_topics=[], sync_topics=['/test_topic'],
                ignore_srv=[], sync_srv=[],
                ignore_type=[],
                ignore_publishers=[], ignore_subscribers=[],
                do_not_sync=[])
        ignore_by_do_no_sync = fi.do_not_sync(
            ['/some_node', '/test_topic', 'SomeType'])
        self.assertFalse(
            ignore_by_do_no_sync, "/test_topic is in sync_topic, but ignored by do not sync")
        ignore = fi.is_ignored_publisher('/some_node', '/test_topic', '')
        self.assertFalse(
            ignore, "/test_topic is in sync_topic, but ignored by filter interface")


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestFilterInterface)
