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

from typing import Any
from typing import Callable
from typing import Union
from typing import Tuple

import argparse
import json
import time
from types import SimpleNamespace

import rospy
from roslib import message
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.runtime_interface import SubscriberEvent
from fkie_mas_pylib.interface.runtime_interface import SubscriberFilter
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket.client import WebSocketClient


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


class MsgEncoder(json.JSONEncoder):
    def __init__(self, *, skipkeys: bool = False, ensure_ascii: bool = True, check_circular: bool = True, allow_nan: bool = True, sort_keys: bool = False, indent: Union[int, str, None] = None, separators: Union[Tuple[str, str], None] = None, default: Union[Callable[..., Any], None] = None,
                 no_arr: bool = True,
                 no_str: bool = True) -> None:
        super().__init__(skipkeys=skipkeys, ensure_ascii=ensure_ascii, check_circular=check_circular,
                         allow_nan=allow_nan, sort_keys=sort_keys, indent=indent, separators=separators, default=default)
        self.no_arr = no_arr
        self.no_str = no_str

    def default(self, obj):
        result = {}
        if isinstance(obj, bytes):
            # obj_bytes = [byte for byte in obj]
            obj_bytes = []
            for byte in obj:
                if len(obj_bytes) >= 10:
                    break
                obj_bytes.append(byte)
            result = ', '.join(map(str, obj_bytes))
            result = result + f'...(of {len(obj)} values)'
        else:
            for key in obj.__slots__:
                skip = False
                if self.no_arr and isinstance(getattr(obj, key), (list, dict, bytes)):
                    skip = True
                if self.no_str and isinstance(getattr(obj, key), str):
                    skip = True
                if skip:
                    result[key] = ""
                else:
                    result[key] = getattr(obj, key)
        return result


class SubscriberNode:

    DEFAULT_WINDOWS_SIZE = 100

    def __init__(self, node_name: str, log_level: int = rospy.INFO, test_env: bool = False):
        self.parser = self._init_arg_parser()
        parsed_args, remaining_args = self.parser.parse_known_args()
        if parsed_args.help:
            return None
        rospy.init_node(node_name, log_level=log_level)
        self._topic = parsed_args.topic
        self._message_type = parsed_args.message_type
        self._count_received = 0
        self._no_data = parsed_args.no_data
        self._no_arr = parsed_args.no_arr
        self._no_str = parsed_args.no_str
        self._hz = parsed_args.hz
        self._window = parsed_args.window
        if self._window == 0:
            self._window = self.DEFAULT_WINDOWS_SIZE
        self._tcp_no_delay = parsed_args.tcp_no_delay
        self._ws_port = parsed_args.ws_port
        self._first_msg_ts = 0

        self._send_ts = 0
        self._latched_messages = []
        # stats parameter
        self._last_received_ts = 0
        self._msg_t0 = -1.
        self._msg_tn = 0
        self._times = []
        self._bytes = []
        self._bws = []

        Log.info(f"start subscriber for {self._topic}[{self._message_type}]")
        self.__msg_class = message.get_message_class(self._message_type)
        if self.__msg_class:
            self.wsClient = WebSocketClient(self._port)
            self.wsClient.subscribe(
                f"ros.subscriber.filter.{self._topic.replace('/', '_')}", self._clb_update_filter)
            self.sub = rospy.Subscriber(
                self._topic, self.__msg_class, self._msg_handle)
            self.subscribe_to(
                f"ros.subscriber.filter.{self._topic.replace('/', '_')}", self._clb_update_filter)
        else:
            raise Exception(
                f"Cannot load message class for [{self._message_type}]. Did you build messages?")

    def __del__(self):
        self.stop()

    def stop(self):
        if hasattr(self, 'wsClient'):
            self.wsClient.shutdown()

    def _init_arg_parser(self) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser()
        parser.add_argument('--ws_port', nargs='?', type=int,
                            required=True,  help='port for websocket server')
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
        parser.add_argument('--window', nargs='?', type=int, default=1,
                            help='window size, in # of messages, for calculating rate.')
        parser.add_argument('--tcp_no_delay', action='store_true',
                            help='use the TCP_NODELAY transport hint when subscribing to topics (Only ROS1).')
        # parser.add_argument('--use_sim_time', type=str2bool, nargs='?', const=True, default=False, help='Enable ROS simulation time (Only ROS2).')
        parser.set_defaults(no_data=False)
        parser.set_defaults(no_arr=False)
        parser.set_defaults(no_str=False)
        parser.set_defaults(tcp_no_delay=False)
        parser.set_defaults(help=False)
        return parser

    def _msg_handle(self, data):
        self._count_received += 1
        self._latched = data._connection_header['latching'] != '0'
        # print(data._connection_header)
        # print(dir(data))
        # print("SIZE", data.__sizeof__())
        print(f"LATCHEND: {data._connection_header['latching'] != '0'}")
        event = SubscriberEvent(self._topic, self._message_type)
        event.latched = self._latched
        if not self._no_data:
            event.data = json.loads(json.dumps(
                data, cls=MsgEncoder, **{"no_arr": self._no_arr, "no_str": self._no_str}))
        event.count = self._count_received
        self._calc_stats(data, event)
        timeouted = self._hz == 0
        if self._hz != 0:
            now = time.time()
            if now - self._send_ts > 1.0 / self._hz:
                self._send_ts = now
                timeouted = True
        if (event.latched and time.time() - self._first_msg_ts < 2.0) or timeouted:
            self.wsClient.publish(
                f"ros.subscriber.event.{self._topic.replace('/', '_')}", json.dumps(event, cls=SelfEncoder), latched=self._latched)

    def _get_message_size(self, msg):
        buff = None
        from io import BytesIO  # Python 3.x
        buff = BytesIO()
        msg.serialize(buff)
        return buff.getbuffer().nbytes

    def _calc_stats(self, msg, event):
        current_time = time.time()
        if current_time - self._last_received_ts > 1:
            pass
        msg_len = self._get_message_size(msg)
        if msg_len > -1:
            event.size = msg_len
            event.size_min = msg_len
            event.size_max = msg_len
        if self._msg_t0 < 0 or self._msg_t0 > current_time:
            self._msg_t0 = current_time
            self._msg_tn = current_time
            self._times = []
            self._bytes = []
        else:
            self._times.append(current_time - self._msg_tn)
            self._msg_tn = current_time

            if msg_len > -1:
                self._bytes.append(msg_len)
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

    def _clb_update_filter(self, json_filter: SubscriberFilter):
        # Convert filter settings into a proper python object
        request = json.loads(json.dumps(json_filter),
                             object_hook=lambda d: SimpleNamespace(**d))
        Log.info(f"update filter for {self._topic}[{self._message_type}]")
        self._no_data = request.no_data
        self._no_arr = request.no_arr
        self._no_str = request.no_str
        self._hz = request.hz
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
