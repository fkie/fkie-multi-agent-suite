# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import argparse
import json
import os
import signal
import sys
import time
import traceback
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from fkie_mas_pylib.defines import ros2_action_nodename_tuple
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.runtime_interface import ActionEvent
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket.client import WebSocketClient
from rclpy.action import ActionClient
from rosidl_runtime_py.utilities import get_action

import fkie_mas_daemon as nmd

from .msg_encoder import MsgEncoder


class RosActionClientLauncher:
    """
    Launches a ROS2 action client node to send goals and forward feedback/result via websocket.
    """

    def __init__(self, test_env=False):
        self.parser = self._init_arg_parser()
        parsed_args, remaining_args = self.parser.parse_known_args()
        if parsed_args.help:
            return None

        self.namespace, self.name = ros2_action_nodename_tuple(parsed_args.action_name)
        # Change terminal name
        print("\33]0;%s\a" % (self.name), end="", flush=True)

        self._port = parsed_args.ws_port
        if os.environ.get("ROS_DISTRO") != "galactic":
            signal.signal(signal.SIGTERM, self.exit_gracefully)
            signal.signal(signal.SIGINT, self.exit_gracefully)

        rclpy.init(args=remaining_args)
        self.ros_node = rclpy.create_node(self.name, namespace=self.namespace)

        self._action_name = parsed_args.action_name
        self._action_type = parsed_args.action_type
        self._ws_port = parsed_args.ws_port
        self._on_shutdown = False
        self._goal_handle = None
        self._goal_id = ""
        self._goal_done = False  # marks a finished goal (result received)

        nmd.ros_node = self.ros_node
        Log.set_ros2_logging_node(self.ros_node)

        self.__action_class = get_action(self._action_type)

        self.wsClient = WebSocketClient(self._port)
        self.wsClient.subscribe("event", self._on_ws_event)

        # Create action client
        self._action_client = ActionClient(self.ros_node, self.__action_class, self._action_name)

        Log.info(f"start ROS action client for {self._action_name}[{self._action_type}]")

        # Parse and send goal from args if provided
        if parsed_args.goal_json:
            self._send_goal_from_json(parsed_args.goal_json)

    def __del__(self):
        self.stop()

    def stop(self):
        if hasattr(self, "wsClient"):
            if self.wsClient:
                self.wsClient.shutdown()
                self.wsClient = None

    def exit_gracefully(self, signum=-1, frame=None):
        if self._on_shutdown:
            return
        self._on_shutdown = True
        Log.info("shutdown action client")
        # Only tries to cancel if the goal is still active
        self._cancel_goal()
        self.stop()
        if rclpy.ok():
            self._action_client.destroy()
            # This stops rclpy.spin() cleanly instead of spinning again
            rclpy.shutdown()
        print("bye!")

    def _on_ws_event(self, msg):
        """Handle websocket events, e.g. cancel request or no more subscribers."""
        if msg.type == "cancel":
            Log.info(f"Cancel request received for action '{self._action_name}'")
            self._cancel_goal()
            self.exit_gracefully()
        if msg.type == "subs" and msg.count == 0:
            feedback_uri = f"ros.action.feedback.{self._action_name.replace('/', '_')}"
            result_uri = f"ros.action.result.{self._action_name.replace('/', '_')}"
            if msg.uri in (feedback_uri, result_uri):
                Log.info(f"No websocket subscriptions for action '{self._action_name}' -> exit!")
                self._cancel_goal()
                self.exit_gracefully()

    def _send_goal_from_json(self, goal_json: str):
        """Parse JSON goal and send it to the action server."""
        try:
            parsed = json.loads(goal_json)
            goal_dict = self._extract_values(parsed)
            Log.info(f"Parsed goal values: {goal_dict}")
            self._send_goal(goal_dict)
        except json.JSONDecodeError as e:
            Log.error(f"Failed to parse goal JSON: {e}")
            self._publish_result("aborted", None, f"Failed to parse goal: {e}")

    # ------------------------------------------------------------------ #
    #  GUI-definition -> plain value dict conversion
    # ------------------------------------------------------------------ #
    def _extract_values(self, parsed: Any) -> dict:
        """Convert the GUI goal-definition structure into a plain value dict.

        The GUI sends a structure like:
          [
            {
              "action_name": "/fibonacci",
              "action_type": "example_interfaces/action/Fibonacci",
              "goal": "<json-string of the goal definition>"
            }
          ]
        where the goal definition itself is:
          {
            "type": "example_interfaces/action/Fibonacci_Goal",
            "name": "",
            "def": [
              {"name": "order", "def": [], "type": "int32",
               "is_array": false, "useNow": false, "value": "5"}
            ],
            "useNow": false
          }

        This method also accepts a plain value dict like {"order": 5}.
        """
        # 1) unwrap a list wrapper -> use the entry that matches our action
        if isinstance(parsed, list):
            selected = None
            for item in parsed:
                if isinstance(item, dict) and item.get("action_name") == self._action_name:
                    selected = item
                    break
            if selected is None and parsed:
                selected = parsed[0]
            parsed = selected

        # 2) unwrap {"goal": "<json-string>"} wrapper
        if isinstance(parsed, dict) and "goal" in parsed:
            goal_field = parsed["goal"]
            if isinstance(goal_field, str):
                goal_field = json.loads(goal_field)
            parsed = goal_field

        # 3) GUI definition object with a "def" list of fields
        if isinstance(parsed, dict) and isinstance(parsed.get("def"), list):
            result = {}
            for field in parsed["def"]:
                name = field.get("name")
                if not name:
                    continue
                result[name] = self._convert_field(field)
            return result

        # 4) already a plain value dict
        if isinstance(parsed, dict):
            return parsed

        # fallback
        return {}

    def _convert_field(self, field: dict) -> Any:
        """Convert a single GUI field definition into its typed value."""
        field_type = field.get("type", "")
        is_array = field.get("is_array", False)
        sub_def = field.get("def")

        # Complex (nested message) type: recurse over its "def" list
        if isinstance(sub_def, list) and sub_def and isinstance(sub_def[0], dict) and "name" in sub_def[0]:
            nested = {}
            for sub in sub_def:
                sub_name = sub.get("name")
                if sub_name:
                    nested[sub_name] = self._convert_field(sub)
            return nested

        # Primitive value(s) stored in "value"
        value = field.get("value")
        if is_array:
            if isinstance(value, str):
                try:
                    value = json.loads(value)
                except json.JSONDecodeError:
                    value = [v.strip() for v in value.split(",") if v.strip() != ""]
            if not isinstance(value, list):
                value = [value] if value is not None else []
            return [self._cast_scalar(field_type, v) for v in value]

        return self._cast_scalar(field_type, value)

    @staticmethod
    def _cast_scalar(field_type: str, value: Any) -> Any:
        """Cast a scalar string value to the correct Python type based on ROS type."""
        if value is None:
            return value
        t = field_type.lower()
        try:
            if any(t.startswith(p) for p in ("int", "uint", "byte", "char")):
                return int(value)
            if t.startswith("float") or t.startswith("double"):
                return float(value)
            if t.startswith("bool"):
                if isinstance(value, bool):
                    return value
                return str(value).lower() in ("1", "true", "t", "yes", "y")
            # string and everything else
            return value
        except (TypeError, ValueError) as e:
            Log.warn(f"Could not cast value '{value}' to type '{field_type}': {e}")
            return value

    # ------------------------------------------------------------------ #

    def _send_goal(self, goal_dict: dict):
        """Send a goal to the action server."""
        Log.info(f"Waiting for action server '{self._action_name}'...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            Log.error(f"Action server '{self._action_name}' not available!")
            self._publish_result("aborted", None, "Action server not available")
            self.exit_gracefully()
            return

        goal_msg = self.__action_class.Goal()
        self._fill_message_from_dict(goal_msg, goal_dict)

        Log.info(f"Sending goal to '{self._action_name}'")
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _fill_message_from_dict(self, msg, values: dict):
        """Recursively fill a ROS message from a dictionary."""
        for field_name, field_value in values.items():
            if not hasattr(msg, field_name):
                Log.warn(f"Message has no field '{field_name}' -> skipped")
                continue
            attr = getattr(msg, field_name)
            if isinstance(field_value, dict) and hasattr(attr, "__slots__"):
                self._fill_message_from_dict(attr, field_value)
            else:
                try:
                    setattr(msg, field_name, field_value)
                except (TypeError, AttributeError) as e:
                    # Important: a silently unset field (e.g. 'order') can
                    # cause the server to abort the goal immediately.
                    Log.error(f"Could not set field '{field_name}'={field_value}: {e}")

    def _goal_response_callback(self, future):
        """Called when the action server accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            Log.warn(f"Goal rejected by action server '{self._action_name}'")
            self._publish_result("rejected", None, "Goal rejected")
            self.exit_gracefully()
            return

        self._goal_handle = goal_handle
        self._goal_id = str(goal_handle.goal_id.uuid.tolist())
        Log.info(f"Goal accepted: {self._goal_id}")

        # Get the result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """Called when feedback is received from the action server."""
        feedback = feedback_msg.feedback
        feedback_data = json.loads(
            json.dumps(feedback, cls=MsgEncoder, **{"no_arr": False, "no_str": False, "array_items_count": 15})
        )
        event = ActionEvent(
            action_name=self._action_name,
            action_type=self._action_type,
            event_type="feedback",
            goal_id=self._goal_id,
            status="executing",
            data=feedback_data,
            timestamp=time.time(),
        )
        self.wsClient.publish(
            f"ros.action.feedback.{self._action_name.replace('/', '_')}", json.dumps(event, cls=SelfEncoder)
        )

    def _result_callback(self, future):
        """Called when the action completes with a result."""
        result = future.result()
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_CANCELING: "canceling",
            GoalStatus.STATUS_EXECUTING: "executing",
            GoalStatus.STATUS_ACCEPTED: "accepted",
            GoalStatus.STATUS_UNKNOWN: "unknown",
        }
        status = status_map.get(result.status, f"unknown({result.status})")
        result_data = None
        if result.result is not None:
            result_data = json.loads(
                json.dumps(result.result, cls=MsgEncoder, **{"no_arr": False, "no_str": False, "array_items_count": 15})
            )
        Log.info(f"Action '{self._action_name}' finished with status: {status}")

        self._goal_done = True
        self._goal_handle = None

        self._publish_result(status, result_data)
        self.exit_gracefully()

    def _publish_result(self, status: str, data, message: str = ""):
        """Publish the action result via websocket."""
        event = ActionEvent(
            action_name=self._action_name,
            action_type=self._action_type,
            event_type="result",
            goal_id=self._goal_id,
            status=status,
            data=data,
            timestamp=time.time(),
        )
        if message:
            event.message = message
        self.wsClient.publish(
            f"ros.action.result.{self._action_name.replace('/', '_')}", json.dumps(event, cls=SelfEncoder)
        )

    def _cancel_goal(self):
        """Cancel the current goal only if it is still active."""
        # Nothing to cancel if the goal already finished or was never accepted
        if self._goal_handle is None or self._goal_done:
            return

        Log.info(f"Canceling goal for '{self._action_name}'")
        try:
            # Do NOT use spin_until_future_complete here:
            # this method may be called from within a callback while the
            # executor is already spinning ("Executor is already spinning").
            # Use a fire-and-forget cancel request instead.
            self._goal_handle.cancel_goal_async()
        except Exception as e:
            Log.warn(f"Failed to cancel goal: {e}")
        self._goal_handle = None

    def _init_arg_parser(self) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser()
        parser.add_argument("--ws_port", nargs="?", type=int, required=True, help="Port for websocket server")
        parser.add_argument(
            "-a", "--action_name", nargs="?", required=True, help="Name of the ROS action (e.g. '/navigate_to_pose')"
        )
        parser.add_argument(
            "-t",
            "--action_type",
            nargs="?",
            required=True,
            help="Type of the ROS action (e.g. 'nav2_msgs/action/NavigateToPose')",
        )
        parser.add_argument("-g", "--goal_json", nargs="?", default=None, help="Goal as JSON string")
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
            # Context already shut down (e.g. after exit_gracefully)
            if "Context must be initialized" in str(e):
                pass
            else:
                raise
        except Exception:
            self.ros_node.get_logger().warning("Start failed: %s" % traceback.format_exc())
            sys.stdout.flush()
            self.exit_gracefully(-1, None)
