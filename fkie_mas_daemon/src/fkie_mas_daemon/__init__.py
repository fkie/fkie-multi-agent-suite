#!/usr/bin/env python
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import argparse
import os
import signal
import sys
import traceback

import rospy

from .daemon import MASDaemon
from fkie_mas_pylib.websocket import ws_port
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
                            default=ws_port(),  help='change port for websocket server')
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
    print("bye!")


def create_subscriber(node_name='mas_subscriber'):
    '''
    Creates a subscriber to forward received messages to websocket server.
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
