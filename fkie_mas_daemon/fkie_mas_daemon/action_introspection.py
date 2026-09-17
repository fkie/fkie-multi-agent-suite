# ****************************************************************************
# Subscribes to an action's *_service_event topics and forwards them via WS.
# ****************************************************************************

import argparse
import json
import os
import signal
import sys
import time
import traceback

import rclpy
from fkie_mas_pylib.defines import ros2_action_introspection_nodename_tuple
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket.client import WebSocketClient
from rosidl_runtime_py.utilities import get_message

import fkie_mas_daemon as nmd

from .msg_encoder import MsgEncoder

# service_msgs/msg/ServiceEventInfo
EVENT_TYPE_MAP = {
    0: "REQUEST_SENT",
    1: "REQUEST_RECEIVED",
    2: "RESPONSE_SENT",
    3: "RESPONSE_RECEIVED",
}

# Mapping of introspection topic suffixes to a "phase"
PHASE_BY_SUFFIX = {
    "/_action/send_goal/_service_event": "send_goal",
    "/_action/get_result/_service_event": "get_result",
    "/_action/cancel_goal/_service_event": "cancel_goal",
}


class ActionIntrospectionEvent:
    """Sent as JSON via WebSocket."""

    def __init__(self, action_name, phase, event_type, sequence_number, client_gid, data, timestamp):
        self.action_name = action_name
        self.phase = phase  # send_goal | get_result | cancel_goal
        self.event_type = event_type  # REQUEST_SENT ...
        self.sequence_number = sequence_number
        self.client_gid = client_gid
        self.data = data  # request/response payload or None
        self.timestamp = timestamp


class RosActionIntrospectionLauncher:
    """
    Starts a ROS2 node that subscribes to an action's introspection
    (_service_event) topics and forwards events via WebSocket.
    """

    def __init__(self, test_env=False):
        self.parser = self._init_arg_parser()
        parsed_args, remaining_args = self.parser.parse_known_args()
        if parsed_args.help:
            return None

        self.namespace, self.name = ros2_action_introspection_nodename_tuple(f"{parsed_args.action_name}")
        print("\33]0;%s\a" % (self.name), end="", flush=True)

        self._port = parsed_args.ws_port
        self._action_name = parsed_args.action_name
        self._action_type = parsed_args.action_type
        self._on_shutdown = False
        self._subscriptions = []

        if os.environ.get("ROS_DISTRO") != "galactic":
            signal.signal(signal.SIGTERM, self.exit_gracefully)
            signal.signal(signal.SIGINT, self.exit_gracefully)

        rclpy.init(args=remaining_args)
        self.ros_node = rclpy.create_node(self.name, namespace=self.namespace)
        nmd.ros_node = self.ros_node
        Log.set_ros2_logging_node(self.ros_node)

        self.wsClient = WebSocketClient(self._port)
        self.wsClient.subscribe("event", self._on_ws_event)

        Log.info(f"start action introspection for {self._action_name}[{self._action_type}]")
        self._subscribe_to_introspection_topics()

    def __del__(self):
        self.stop()

    def stop(self):
        if hasattr(self, "wsClient") and self.wsClient:
            self.wsClient.shutdown()
            self.wsClient = None

    def exit_gracefully(self, signum=-1, frame=None):
        if self._on_shutdown:
            return
        self._on_shutdown = True
        Log.info("shutdown action introspection")
        self.stop()
        if rclpy.ok():
            rclpy.shutdown()
        print("bye!")

    def _on_ws_event(self, msg):
        """Handles cancel / no-more-subscribers events from the client."""
        if msg.type == "cancel":
            self.exit_gracefully()
        if msg.type == "subs" and msg.count == 0:
            uri = f"ros.action.introspection.{self._action_name.replace('/', '_')}"
            if getattr(msg, "uri", None) == uri:
                Log.info(f"No subscriptions for '{self._action_name}' introspection -> exit!")
                self.exit_gracefully()

    def _subscribe_to_introspection_topics(self):
        """Dynamically resolves topic types and subscribes to the _service_event topics."""
        # Short wait to give topic discovery time
        for _ in range(50):
            topics = dict(self.ros_node.get_topic_names_and_types())
            found_any = False
            for suffix, phase in PHASE_BY_SUFFIX.items():
                topic = f"{self._action_name}{suffix}"
                if topic in topics and topic not in [s[0] for s in self._subscriptions]:
                    type_str = topics[topic][0]
                    self._create_subscription(topic, type_str, phase)
                    found_any = True
            if len(self._subscriptions) >= len(PHASE_BY_SUFFIX):
                break
            time.sleep(0.1)

        if not self._subscriptions:
            Log.warn(
                f"No introspection topics found for '{self._action_name}'. "
                "Is introspection enabled (State METADATA/CONTENTS)?"
            )

    def _create_subscription(self, topic: str, type_str: str, phase: str):
        try:
            msg_class = get_message(type_str)
        except Exception as e:
            Log.error(f"Could not resolve type '{type_str}' for '{topic}': {e}")
            return

        def _cb(msg, _phase=phase):
            self._on_service_event(msg, _phase)

        sub = self.ros_node.create_subscription(msg_class, topic, _cb, 10)
        self._subscriptions.append((topic, sub))
        Log.info(f"subscribed introspection topic '{topic}' [{type_str}] ({phase})")

    def _guid_arr_to_str(self, gid: list[int]) -> str:
        return ".".join(f"{c:02X}" for c in gid)

    def _on_service_event(self, msg, phase: str):
        try:
            info = msg.info
            event_type = EVENT_TYPE_MAP.get(info.event_type, f"unknown({info.event_type})")
            _gid = getattr(info, "client_gid", None)
            client_gid = self._guid_arr_to_str(_gid) if _gid is not None else []
            seq = int(getattr(info, "sequence_number", 0))

            # request/response are arrays (empty in METADATA state)
            payload = None
            request = list(getattr(msg, "request", []) or [])
            response = list(getattr(msg, "response", []) or [])
            content = request or response
            if content:
                payload = json.loads(
                    json.dumps(
                        content[0], cls=MsgEncoder, **{"no_arr": False, "no_str": False, "array_items_count": 50}
                    )
                )

            event = ActionIntrospectionEvent(
                action_name=self._action_name,
                phase=phase,
                event_type=event_type,
                sequence_number=seq,
                client_gid=client_gid,
                data=payload,
                timestamp=time.time(),
            )
            self.wsClient.publish(
                f"ros.action.introspection.{self._action_name.replace('/', '_')}",
                json.dumps(event, cls=SelfEncoder),
            )
        except Exception as e:
            Log.error(f"Error handling service event: {e}\n{traceback.format_exc()}")

    def _init_arg_parser(self) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser()
        parser.add_argument("--ws_port", nargs="?", type=int, required=True, help="Port for websocket server")
        parser.add_argument(
            "-a", "--action_name", nargs="?", required=True, help="Name of the ROS action (e.g. '/fibonacci')"
        )
        parser.add_argument(
            "-t",
            "--action_type",
            nargs="?",
            required=True,
            help="Type of the action (e.g. 'example_interfaces/action/Fibonacci')",
        )
        parser.set_defaults(help=False)
        return parser

    def spin(self):
        if self._on_shutdown:
            return
        try:
            rclpy.spin(self.ros_node)
        except KeyboardInterrupt:
            self.exit_gracefully(-1, None)
        except rclpy.executors.ExternalShutdownException:
            pass
        except RuntimeError as e:
            if "Context must be initialized" not in str(e):
                raise
        except Exception:
            self.ros_node.get_logger().warning("Start failed: %s" % traceback.format_exc())
            sys.stdout.flush()
            self.exit_gracefully(-1, None)
