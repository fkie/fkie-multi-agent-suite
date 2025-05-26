# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import gc
import os
import argparse
import json
import signal
import sys
import time
import traceback
from typing import Optional
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.runtime_interface import SubscriberEvent
from fkie_mas_pylib.interface.runtime_interface import SubscriberFilter
from fkie_mas_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_mas_pylib.websocket import ws_port
from fkie_mas_pylib.websocket.client import WebSocketClient
from fkie_mas_pylib import formats
import fkie_mas_daemon as nmd
from fkie_mas_pylib.logging.logging import Log
from .msg_encoder import MsgEncoder


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


class RosSubscriberLauncher:
    '''
    Launches the ROS node to forward a topic subscription.
    '''

    DEFAULT_WINDOWS_SIZE = 5000

    def __init__(self, test_env=False):
        self.parser = self._init_arg_parser()
        # change terminal name
        parsed_args, remaining_args = self.parser.parse_known_args()
        if parsed_args.help:
            return None
        self.namespace, self.name = ros2_subscriber_nodename_tuple(
            parsed_args.topic)
        print('\33]0;%s\a' % (self.name), end='', flush=True)
        self._port = parsed_args.ws_port
        if os.environ.get('ROS_DISTRO') != 'galactic':
            signal.signal(signal.SIGTERM, self.exit_gracefully)
            signal.signal(signal.SIGINT, self.exit_gracefully)
        rclpy.init(args=remaining_args)
        # NM_NAMESPACE
        self.ros_node = rclpy.create_node(self.name, namespace=self.namespace)
        self._topic = parsed_args.topic
        self._message_type = parsed_args.message_type
        self._count_received = 0
        self._no_data = parsed_args.no_data
        self._no_arr = parsed_args.no_arr
        self._no_str = parsed_args.no_str
        self._hz = parsed_args.hz
        self._window = parsed_args.window
        self._array_items_count = parsed_args.array_items_count
        if self._window == 0:
            self._window = self.DEFAULT_WINDOWS_SIZE
        self._tcp_no_delay = parsed_args.tcp_no_delay
        self._ws_port = parsed_args.ws_port
        self._parsed_args = parsed_args

        self._send_ts = 0
        self._latched_messages = []
        # stats parameter
        self._last_received_ts = 0
        self._msg_t0 = -1.
        self._msg_tn = 0
        self._times = []
        self._bytes = []
        self._bws = []

        nmd.ros_node = self.ros_node
        # set loglevel to DEBUG
        # nmd.ros_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # get a reference to the global node for logging
        Log.set_ros2_logging_node(self.ros_node)

        Log.info(f"start ROS subscriber for {self._topic}[{self._message_type}]")
        self.__msg_class = get_message(self._message_type)
        qos_state_profile = self.choose_qos(self._parsed_args, self._topic)
        # self.sub = nmd.ros_node.create_subscription(
        #     self.__msg_class, self._topic, self._msg_handle, qos_profile=qos_state_profile)
        self.sub_raw = nmd.ros_node.create_subscription(
            self.__msg_class, self._topic, self._msg_handle_raw, qos_profile=qos_state_profile, raw=True)
        self.wsClient = WebSocketClient(self._port)
        # qos_state_profile = QoSProfile(depth=100,
        #                                # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        #                                # history=QoSHistoryPolicy.KEEP_LAST,
        #                                # reliability=QoSReliabilityPolicy.RELIABLE)
        #                                )
        Log.info(f"subscribe to ROS topic: {self._topic} [{self.__msg_class}]")
        self.wsClient.subscribe(
            f"ros.subscriber.filter.{self._topic.replace('/', '_')}", self._clb_update_filter)

    def __del__(self):
        self.stop()

    def stop(self):
        if hasattr(self, 'wsClient'):
            if self.wsClient:
                self.wsClient.shutdown()
                self.wsClient = None

    def exit_gracefully(self, signum, frame):
        print('shutdown rclpy')
        self.sub_raw.destroy()
        self.stop()
        if rclpy.ok():
            rclpy.shutdown()
        print('bye!')

    # from https://github.com/ros2/ros2cli/blob/rolling/ros2topic/ros2topic/verb/echo.py

    def profile_configure_short_keys(self,
                                     profile: rclpy.qos.QoSProfile = None, reliability: Optional[str] = None,
                                     durability: Optional[str] = None, depth: Optional[int] = None, history: Optional[str] = None,
                                     liveliness: Optional[str] = None, liveliness_lease_duration_s: Optional[int] = None,
                                     ) -> rclpy.qos.QoSProfile:
        """Configure a QoSProfile given a profile, and optional overrides."""
        if history:
            profile.history = rclpy.qos.QoSHistoryPolicy.get_from_short_key(
                history)
        if durability:
            profile.durability = rclpy.qos.QoSDurabilityPolicy.get_from_short_key(
                durability)
        if reliability:
            profile.reliability = rclpy.qos.QoSReliabilityPolicy.get_from_short_key(
                reliability)
        if liveliness:
            profile.liveliness = rclpy.qos.QoSLivelinessPolicy.get_from_short_key(
                liveliness)
        if liveliness_lease_duration_s and liveliness_lease_duration_s >= 0:
            profile.liveliness_lease_duration = Duration(
                seconds=liveliness_lease_duration_s)
        if depth and depth >= 0:
            profile.depth = depth
        else:
            if (profile.durability == rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
                    and profile.depth == 0):
                profile.depth = 1

    # from https://github.com/ros2/ros2cli/blob/rolling/ros2topic/ros2topic/verb/echo.py
    def qos_profile_from_short_keys(self,
                                    preset_profile: str, reliability: Optional[str] = None, durability: Optional[str] = None,
                                    depth: Optional[int] = None, history: Optional[str] = None, liveliness: Optional[str] = None,
                                    liveliness_lease_duration_s: Optional[float] = None,
                                    ) -> rclpy.qos.QoSProfile:
        """Construct a QoSProfile given the name of a preset, and optional overrides."""
        # Build a QoS profile based on user-supplied arguments
        profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(
            preset_profile)
        self.profile_configure_short_keys(
            profile, reliability, durability, depth, history, liveliness, liveliness_lease_duration_s)
        return profile

    # from https://github.com/ros2/ros2cli/blob/rolling/ros2topic/ros2topic/verb/echo.py
    def choose_qos(self, args, topic):
        if (args.qos_reliability is not None or
                args.qos_durability is not None or
                args.qos_depth is not None or
                args.qos_history is not None or
                args.qos_liveliness is not None or
                args.qos_liveliness_lease_duration_seconds is not None):

            return self.qos_profile_from_short_keys(
                args.qos_profile,
                reliability=args.qos_reliability,
                durability=args.qos_durability,
                depth=args.qos_depth,
                history=args.qos_history,
                liveliness=args.qos_liveliness,
                liveliness_lease_duration_s=args.qos_liveliness_lease_duration_seconds)

        qos_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(
            args.qos_profile)
        reliability_reliable_endpoints_count = 0
        durability_transient_local_endpoints_count = 0

        pubs_info = self.ros_node.get_publishers_info_by_topic(topic)
        publishers_count = len(pubs_info)
        if publishers_count == 0:
            return qos_profile

        for info in pubs_info:
            if (info.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE):
                reliability_reliable_endpoints_count += 1
            if (info.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL):
                durability_transient_local_endpoints_count += 1

        # If all endpoints are reliable, ask for reliable
        if reliability_reliable_endpoints_count == publishers_count:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        else:
            if reliability_reliable_endpoints_count > 0:
                print(
                    'Some, but not all, publishers are offering '
                    'QoSReliabilityPolicy.RELIABLE. Falling back to '
                    'QoSReliabilityPolicy.BEST_EFFORT as it will connect '
                    'to all publishers'
                )
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # If all endpoints are transient_local, ask for transient_local
        if durability_transient_local_endpoints_count == publishers_count:
            qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        else:
            if durability_transient_local_endpoints_count > 0:
                print(
                    'Some, but not all, publishers are offering '
                    'QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to '
                    'QoSDurabilityPolicy.VOLATILE as it will connect '
                    'to all publishers'
                )
            qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        return qos_profile

    def spin(self):
        try:
            rclpy.spin(self.ros_node)
        except KeyboardInterrupt:
            self.exit_gracefully(-1, None)
        except rclpy.executors.ExternalShutdownException:
            pass
        except Exception:
            # on load error the process will be killed to notify user
            # in node_manager about error
            self.ros_node.get_logger().warning('Start failed: %s' %
                                               traceback.format_exc())
            sys.stdout.write(traceback.format_exc())
            sys.stdout.flush()
            # TODO: how to notify user in node manager about start errors
            # os.kill(os.getpid(), signal.SIGKILL)
            self.exit_gracefully(-1, None)

    # from https://github.com/ros2/ros2cli/blob/rolling/ros2topic/ros2topic/api/__init__.py
    def add_qos_arguments(self, parser: argparse.ArgumentParser, subscribe_or_publish: str, default_profile_str):
        parser.add_argument(
            '--qos-profile',
            choices=rclpy.qos.QoSPresetProfiles.short_keys(),
            help=(
                f'Quality of service preset profile to {subscribe_or_publish} with'
                f' (default: {default_profile_str})'),
            default=default_profile_str)
        default_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(
            default_profile_str)
        parser.add_argument(
            '--qos-depth', metavar='N', type=int,
            help=(
                f'Queue size setting to {subscribe_or_publish} with '
                '(overrides depth value of --qos-profile option)'))
        parser.add_argument(
            '--qos-history',
            choices=rclpy.qos.QoSHistoryPolicy.short_keys(),
            help=(
                f'History of samples setting to {subscribe_or_publish} with '
                '(overrides history value of --qos-profile option, default: '
                f'{default_profile.history.short_key})'))
        parser.add_argument(
            '--qos-reliability',
            choices=rclpy.qos.QoSReliabilityPolicy.short_keys(),
            help=(
                f'Quality of service reliability setting to {subscribe_or_publish} with '
                '(overrides reliability value of --qos-profile option, default: '
                'Compatible profile with running endpoints )'))
        parser.add_argument(
            '--qos-durability',
            choices=rclpy.qos.QoSDurabilityPolicy.short_keys(),
            help=(
                f'Quality of service durability setting to {subscribe_or_publish} with '
                '(overrides durability value of --qos-profile option, default: '
                'Compatible profile with running endpoints )'))
        parser.add_argument(
            '--qos-liveliness',
            choices=rclpy.qos.QoSLivelinessPolicy.short_keys(),
            help=(
                f'Quality of service liveliness setting to {subscribe_or_publish} with '
                '(overrides liveliness value of --qos-profile option'))
        parser.add_argument(
            '--qos-liveliness-lease-duration-seconds',
            type=float,
            help=(
                f'Quality of service liveliness lease duration setting to {subscribe_or_publish} '
                'with (overrides liveliness lease duration value of --qos-profile option'))

    def _init_arg_parser(self) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser()
        parser.add_argument('--ws_port', nargs='?', type=int,
                            required=True,  help='port for ws server')
        parser.add_argument('-t', '--topic', nargs='?', required=True,
                            help="Name of the ROS topic to listen to (e.g. '/chatter')")
        parser.add_argument("-m", "--message_type", nargs='?', required=True,
                            help="Type of the ROS message (e.g. 'std_msgs/msg/String')")
        parser.add_argument('--no_data', action='store_true',
                            help='Report only statistics without message content.')
        parser.add_argument('--no_arr', action='store_true',
                            help='Exclude arrays.')
        parser.add_argument('--no_str', action='store_true',
                            help='Exclude string fields.')
        parser.add_argument('--hz', nargs='?', type=int, default=1,
                            help='Rate to forward messages. Ignored on latched topics. Disabled by 0.')
        parser.add_argument('--window', nargs='?', type=int, default=0,
                            help='window size, in # of messages, for calculating rate.')
        parser.add_argument('--array_items_count', nargs='?', type=int, default=15,
                            help='Maximum array length in messages reported to the gui')
        parser.add_argument('--tcp_no_delay', action='store_true',
                            help='use the TCP_NODELAY transport hint when subscribing to topics (Only ROS1).')
        self.add_qos_arguments(parser, 'subscribe', 'sensor_data')
        # parser.add_argument('--use_sim_time', type=str2bool, nargs='?', const=True, default=False, help='Enable ROS simulation time (Only ROS2).')
        parser.set_defaults(no_data=False)
        parser.set_defaults(no_arr=False)
        parser.set_defaults(no_str=False)
        parser.set_defaults(tcp_no_delay=False)
        parser.set_defaults(help=False)
        return parser

    def get_obj_size(self, obj):
        marked = {id(obj)}
        obj_q = [obj]
        sz = 0

        while obj_q:
            sz += sum(map(sys.getsizeof, obj_q))

            # Lookup all the object referred to by the object in obj_q.
            # See: https://docs.python.org/3.7/library/gc.html#gc.get_referents
            all_refr = ((id(o), o) for o in gc.get_referents(*obj_q))

            # Filter object that are already marked.
            # Using dict notation will prevent repeated objects.
            new_refr = {o_id: o for o_id, o in all_refr if o_id not in marked and not isinstance(o, type)}

            # The new obj_q will be the ones that were not marked,
            # and we will update marked with their ids so we will
            # not traverse them again.
            obj_q = new_refr.values()
            marked.update(new_refr.keys())

        return sz

    async def _msg_handle_raw(self, data):
        msg_size = len(data)
        msg = deserialize_message(data, self.__msg_class)
        self._count_received += 1
        self._latched = False
        event = SubscriberEvent(self._topic, self._message_type)
        event.latched = self._latched
        if not self._no_data:
            event.data = json.loads(json.dumps(
                msg, cls=MsgEncoder, **{"no_arr": self._no_arr, "no_str": self._no_str, "array_items_count": self._array_items_count}))
        json_msg_size = self.get_obj_size(event.data)
        if json_msg_size > 1048576:
            hints = ""
            if not self._no_arr:
                hints += " Try to disable lists."
            if self._array_items_count == 0 or self._array_items_count > 1:
                hints += " Try to decrease count of displayed array values, current count: {self._array_items_count}."
            if not self._no_str:
                hints += " Try to disable string."
            event.data = {"error": f"json message is to big > 1 MByte ({formats.sizeof_fmt(json_msg_size)}).{hints}"}
        event.count = self._count_received
        self._calc_stats(msg_size, event)
        timeouted = self._hz == 0
        if self._hz != 0:
            now = time.time()
            if now - self._send_ts > 1.0 / self._hz:
                self._send_ts = now
                timeouted = True
        if event.latched or timeouted:
            self.wsClient.publish(
                f"ros.subscriber.event.{self._topic.replace('/', '_')}", json.dumps(event, cls=SelfEncoder), latched=self._latched)

    def _calc_stats(self, msg_size, event):
        current_time = time.time()
        if current_time - self._last_received_ts > 1:
            pass
        if msg_size > -1:
            event.size = msg_size
            event.size_min = msg_size
            event.size_max = msg_size
        if self._msg_t0 < 0 or self._msg_t0 > current_time:
            self._msg_t0 = current_time
            self._msg_tn = current_time
            self._times = []
            self._bytes = []
        else:
            self._times.append(current_time - self._msg_tn)
            self._msg_tn = current_time

            if msg_size > -1:
                self._bytes.append(msg_size)
            if len(self._bytes) > self._window:
                self._bytes.pop(0)
            if len(self._times) > self._window:
                self._times.pop(0)

            sum_times = sum(self._times)

            if self._bytes:
                sum_bytes = sum(self._bytes)
                if sum_times > 3:
                    event.bw = float(sum_bytes) / float(sum_times)
                    self._bws.append(event.bw)
                    if len(self._bws) > self._window:
                        self._bws.pop(0)
                    event.bw_min = min(self._bws)
                    event.bw_max = max(self._bws)
                event.size_min = min(self._bytes)
                event.size_max = max(self._bytes)

        # the code from ROS rostopic
        n = len(self._times)
        if n > 1:
            avg = sum_times / n
            event.rate = 1. / avg if avg > 0. else 0

        # # min and max
        # if self.SHOW_JITTER or self.show_only_rate:
        #     max_delta = max(self.times)
        #     min_delta = min(self.times)
        #     message_jitter = "jitter[ min: %.3fs   max: %.3fs ]" % (
        #         min_delta, max_delta)
        # # std dev
        # self.last_printed_count = self.message_count
        # if self.SHOW_STD_DEV or self.show_only_rate:
        #     std_dev = math.sqrt(
        #         sum((x - mean) ** 2 for x in self.times) / n)
        #     message_std_dev = "std dev: %.5fs" % (std_dev)
        # if self.SHOW_WINDOW_SIZE or self.show_only_rate:
        #     message_window = "window: %s" % (n + 1)

        self._last_received_ts = current_time

    def _clb_update_filter(self, request: SubscriberFilter):
        Log.info(f"update filter for {self._topic}[{self._message_type}]")
        self._no_data = request.no_data
        self._no_arr = request.no_arr
        self._no_str = request.no_str
        self._hz = request.hz
        if hasattr(request, "arrayItemsCount"):
            self._array_items_count = request.arrayItemsCount
        if self._window != request.window:
            self._window = request.window
            if self._window == 0:
                self._window = self.DEFAULT_WINDOWS_SIZE
            while len(self._bytes) > self._window:
                self._bytes.pop(0)
            while len(self._times) > self._window:
                self._times.pop(0)
            while len(self._bws) > self._window:
                self._bws.pop(0)
