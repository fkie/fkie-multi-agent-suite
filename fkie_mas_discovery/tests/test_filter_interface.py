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
