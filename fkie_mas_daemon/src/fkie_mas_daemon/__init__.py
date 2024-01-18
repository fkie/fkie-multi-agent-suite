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


import argparse
import os
import signal
import sys
import traceback

import rospy

from .daemon import MASDaemon
from fkie_mas_pylib.crossbar import server
from fkie_mas_pylib.system.screen import test_screen
from fkie_mas_pylib.logging.logging import Log
from .subscriber_node import SubscriberNode


def set_terminal_name(name):
    '''
    Change the terminal name.

    :param str name: New name of the terminal
    '''
    sys.stdout.write("".join(["\x1b]2;", name, "\x07"]))


def set_process_name(name):
    '''
    Change the process name.

    :param str name: New process name
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


def init_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--load", nargs=1, help="loads given file on start;"
                                                      " statements like pkg://PACKAGE/subfolder/LAUNCH are resolved to absolute path;"
                                                      " comma separated for multiple files")
    parser.add_argument("-a", "--autostart", nargs=1, help="loads given file on start and launch nodes after load launch file;"

                                                           " statements like pkg://PACKAGE/subfolder/LAUNCH are resolved to absolute path;"
                                                           " comma separated for multiple files")
    parser.add_argument('--port', nargs='?', type=int,
                            default=server.port(),  help='change port for crossbar server')
    return parser



def start_server(node_name='mas_daemon'):
    '''
    Creates and runs the ROS node
    '''
    # setup the loglevel
    log_level = rospy.DEBUG
    try:
        log_level = getattr(rospy, rospy.get_param(
            '/%s/log_level' % node_name, "INFO"))
    except Exception as e:
        print("Error while set the log level: %s\n->INFO level will be used!" % e)
    rospy.init_node(node_name, log_level=log_level)
    set_terminal_name(node_name)
    set_process_name(node_name)
    # load parameter
    parser = init_arg_parser()
    args = rospy.myargv(argv=sys.argv)
    parsed_args = parser.parse_args(args[1:])
    load_files = []
    if parsed_args.load:
        load_files = parsed_args.load[0].split(',')
    start_files = []
    if parsed_args.autostart:
        start_files = parsed_args.autostart[0].split(',')
    try:
        test_screen()
    except Exception as e:
        Log.error("No SCREEN available! You can't launch nodes.")
    try:
        launch_manager = MASDaemon()
        launch_manager.start(parsed_args.port)
        # load launch file from parameter
        for lfile in load_files:
            launch_manager.load_launch_file(lfile, autostart=False)
        for sfile in start_files:
            launch_manager.load_launch_file(sfile, autostart=True)
        rospy.spin()
        launch_manager.shutdown()
    except Exception:
        # on load error the process will be killed to notify user in node_manager
        # about error
        Log.warn("Start server failed: %s", traceback.format_exc())
        sys.stdout.write(traceback.format_exc())
        sys.stdout.flush()
        os.kill(os.getpid(), signal.SIGKILL)


def create_subscriber(node_name='node_manager_subscriber'):
    '''
    Creates a subscriber to forward received messages to crossbar server.
    '''
    # setup the loglevel
    log_level = rospy.DEBUG
    try:
        log_level = getattr(rospy, rospy.get_param(
            '/%s/log_level' % node_name, "INFO"))
    except Exception as e:
        print("Error while set the log level: %s\n->INFO level will be used!" % e)
    set_terminal_name(node_name)
    set_process_name(node_name)
    try:
        subscriber_node = SubscriberNode(node_name, log_level=log_level)
        rospy.spin()
        subscriber_node.stop()
    except Exception:
        # on load error the process will be killed to notify user in node_manager
        # about error
        Log.error("Start subscriber node: %s", traceback.format_exc())
        os.kill(os.getpid(), signal.SIGKILL)
