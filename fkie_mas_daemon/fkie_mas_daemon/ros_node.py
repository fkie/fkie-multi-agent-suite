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
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.client import SrvType
from rclpy.client import SrvTypeRequest
from rclpy.client import SrvTypeResponse
from fkie_mas_daemon.server import Server
from fkie_mas_pylib.defines import NM_DAEMON_NAME
from fkie_mas_pylib.defines import NM_NAMESPACE
from fkie_mas_pylib.system.host import ros_host_suffix
from fkie_mas_pylib.system.screen import test_screen
from fkie_mas_pylib.websocket import ws_port
import fkie_mas_daemon as nmd
from fkie_mas_pylib.logging.logging import Log


class RosNodeLauncher(object):
    '''
    Launches the ROS node.
    Sets global parameter `ros_node` while initialization.
    '''

    def __init__(self):
        self.ros_domain_id = 0
        self.parser = self._init_arg_parser()
        self.name = NM_DAEMON_NAME
        # change terminal name
        print('\33]0;%s\a' % (self.name), end='', flush=True)
        parsed_args, remaining_args = self.parser.parse_known_args()
        self._displayed_name = parsed_args.name
        self._port = parsed_args.port
        self._load = parsed_args.load
        self._autostart = parsed_args.autostart
        if 'ROS_DOMAIN_ID' in os.environ:
            self.ros_domain_id = int(os.environ['ROS_DOMAIN_ID'])
            # TODO: switch domain id
            # os.environ.pop('ROS_DOMAIN_ID')
        if os.environ.get('ROS_DISTRO') != 'galactic':
            signal.signal(signal.SIGTERM, self.exit_gracefully)
            signal.signal(signal.SIGINT, self.exit_gracefully)
        rclpy.init(args=remaining_args)
        try:
            self.ros_node = rclpy.create_node(self.name, namespace=NM_NAMESPACE, enable_logger_service=True)
        except TypeError:
            self.ros_node = rclpy.create_node(self.name, namespace=NM_NAMESPACE)

        nmd.ros_node = self.ros_node
        # self.executor = MultiThreadedExecutor(num_threads=5)
        # self.executor.add_node(self.ros_node)
        # set loglevel to DEBUG
        # nmd.ros_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # get a reference to the global node for logging
        Log.set_ros2_logging_node(nmd.ros_node)

        # test for screen after ros_node log module is available.
        self._run_tests()
        # nmd.ros_node.declare_parameter('force_insecure', value=False, descriptor=ParameterDescriptor(description='Ignore security options and use insecure channel'), ignore_override = False)
        self.server = Server(
            self.ros_node, default_domain_id=self.ros_domain_id)
        self.success_start = False

    def exit_gracefully(self, signum, frame):
        print('shutdown own server')
        self.server.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        print('bye!')

    def exception_handler(self, loop, error):
        pass

    def spin(self):
        try:
            # start server and load launch files provided by arguments
            self.success_start = self.server.start(
                self._port, displayed_name=self._displayed_name)
            if self.success_start:
                self._load_launches()
                # self.executor.spin()
                rclpy.spin(self.ros_node)
                # rclpy.spin(self.ros_node)
        except KeyboardInterrupt:
            self.exit_gracefully(-1, None)
        except rclpy.executors.ExternalShutdownException as error:
            sys.stdout.write(f"ExternalShutdownException: {error}")
            sys.stdout.flush()
        except Exception:
            import traceback
            # on load error the process will be killed to notify user
            # in node_manager about error
            self.ros_node.get_logger().warning('Start server failed: %s' %
                                               traceback.format_exc())
            sys.stdout.write(traceback.format_exc())
            sys.stdout.flush()
            # TODO: how to notify user in node manager about start errors
            # os.kill(os.getpid(), signal.SIGKILL)
            self.exit_gracefully(-1, None)

    def _run_tests(self):
        try:
            test_screen()
        except Exception:
            import traceback
            print(traceback.format_exc())
            self.ros_node.get_logger().error('No SCREEN available! You cannot launch nodes.')

    def _init_arg_parser(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('-l', '--load', nargs=1, help='loads given file on start;'
                            ' statements like pkg://PACKAGE/subfolder/LAUNCH are resolved to absolute path;'
                            ' comma separated for multiple files')
        parser.add_argument('-a', '--autostart', nargs=1, help='loads given file on start and launch nodes after load launch file;'
                            ' statements like pkg://PACKAGE/subfolder/LAUNCH are resolved to absolute path;'
                            ' comma separated for multiple files')
        parser.add_argument('--name', nargs='?', type=str, default='',
                            help='changes the displayed name of the daemon. Default: hostname')
        parser.add_argument('--port', nargs='?', type=int,
                            default=ws_port(),  help='change port for WebSocket server')
        return parser

    def _load_launches(self):
        load_files = []
        if self._load:
            load_files = self._load[0].split(',')
        start_files = []
        if self._autostart:
            start_files = self._autostart[0].split(',')
        for load_file in load_files:
            self.server.load_launch_file(load_file, autostart=False)
        for start_file in start_files:
            self.server.load_launch_file(start_file, autostart=True)

    def call_service(self, srv_name: str, srv_type: SrvType, request: SrvTypeRequest, timeout_sec: float = 1.0) -> SrvTypeResponse:
        """
        Make a service request and wait for the result.

        :param srv_name: The service name.
        :param srv_type: The service type.
        :param request: The service request.
        :return: The service response.
        """
        client = self.ros_node.create_client(srv_type, srv_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise Exception(f"Service '{srv_name}' of type '{srv_type}' not available")

            event = threading.Event()

            def unblock(future):
                nonlocal event
                event.set()

            future = client.call_async(request)
            future.add_done_callback(unblock)

            event.wait(timeout=timeout_sec)
            if future.exception() is not None:
                raise future.exception()
            return future.result()
        finally:
            self.ros_node.destroy_client(srv_name)

    def call_action(self, srv_name: str, srv_type: SrvType, request: SrvTypeRequest, timeout_sec: float = 1.0) -> SrvTypeResponse:
        [srv_name, action_type] = srv_name.split("/_action/")
        if action_type != "send_goal":
            raise Exception(f"Calling action service '{action_type}' not supported!")

        from action_msgs.msg import GoalStatus
        result = GoalStatus()
        action_client = ActionClient(self.ros_node, srv_type, srv_name)
        try:
            if not action_client.wait_for_server(timeout_sec):
                raise Exception(f"Action '{srv_name}' of type '{srv_type}' not available")
            event = threading.Event()

            def unblock(future):
                nonlocal event
                event.set()

            send_goal_future = None
            send_goal_future = action_client.send_goal_async(request)
            send_goal_future.add_done_callback(unblock)
            if send_goal_future:
                event.wait(timeout=timeout_sec)
                if send_goal_future.exception() is not None:
                    raise send_goal_future.exception()
                goal_handle = send_goal_future.result()
                result.status = goal_handle.status
                if goal_handle.accepted:
                    result.goal_info.goal_id = goal_handle.goal_id
                    result.goal_info.stamp = goal_handle.stamp
            return result
        finally:
            action_client.destroy()
