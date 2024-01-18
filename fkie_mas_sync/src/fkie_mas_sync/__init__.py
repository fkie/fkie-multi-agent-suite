#!/usr/bin/env python
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


import sys

import roslib
import rospy

from . import master_sync

PROCESS_NAME = "mas_sync"


def set_terminal_name(name):
    '''
    Change the terminal name.
    @param name: New name of the terminal
    @type name:  C{str}
    '''
    sys.stdout.write("\x1b]2;%s\x07" % name)


def set_process_name(name):
    '''
    Change the process name.
    @param name: New process name
    @type name:  C{str}
    '''
    try:
        from ctypes import cdll, byref, create_string_buffer
        libc = cdll.LoadLibrary('libc.so.6')
        buff = create_string_buffer(len(name) + 1)
        buff.value = name
        libc.prctl(15, byref(buff), 0, 0, 0)
    except Exception:
        try:
            import setproctitle
            setproctitle.setproctitle(name)
        except Exception:
            pass


def main():
    '''
    Creates and runs the ROS node.
    '''
    # setup the loglevel
    try:
        log_level = getattr(rospy, rospy.get_param(
            '/%s/log_level' % PROCESS_NAME, "INFO"))
    except Exception as e:
        print("Error while set the log level: %s\n->INFO level will be used!" % e)
        log_level = rospy.INFO
    rospy.init_node(PROCESS_NAME, log_level=log_level)
    set_terminal_name(PROCESS_NAME)
    set_process_name(PROCESS_NAME)
    # time to initialize the topics to receive these in rxconsole
    discoverer = master_sync.Main()
    if not rospy.is_shutdown():
        rospy.spin()
