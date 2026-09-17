import argparse
import json
import signal
import time
import traceback

import rclpy
from fkie_mas_pylib.defines import ros2_service_introspection_nodename_tuple
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket.client import WebSocketClient
from rosidl_runtime_py.utilities import get_message

from .msg_encoder import MsgEncoder

EVENT_TYPE_MAP = {
    0: "REQUEST_SENT",
    1: "REQUEST_RECEIVED",
    2: "RESPONSE_SENT",
    3: "RESPONSE_RECEIVED",
}


class ServiceIntrospectionEvent:
    def __init__(self, service_name, event_type, sequence_number, client_gid, data, timestamp):
        self.service_name = service_name
        self.event_type = event_type
        self.sequence_number = sequence_number
        self.client_gid = client_gid
        self.data = data
        self.timestamp = timestamp


class RosServiceIntrospectionLauncher:
    def __init__(self):
        self.parser = self._init_arg_parser()
        parsed_args, remaining_args = self.parser.parse_known_args()

        self.namespace, self.name = ros2_service_introspection_nodename_tuple(parsed_args.service_name)

        self._port = parsed_args.ws_port
        self._service_name = parsed_args.service_name
        self._service_type = parsed_args.service_type
        self._on_shutdown = False
        self._subscription = None

        signal.signal(signal.SIGTERM, self.exit_gracefully)
        signal.signal(signal.SIGINT, self.exit_gracefully)

        rclpy.init(args=remaining_args)
        self.ros_node = rclpy.create_node(self.name, namespace=self.namespace)

        self.wsClient = WebSocketClient(self._port)
        self.wsClient.subscribe("event", self._on_ws_event)

        Log.info(f"start service introspection for {self._service_name}[{self._service_type}]")
        self._subscribe_to_service_event_topic()

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
        Log.info("shutdown service introspection")
        self.stop()
        if rclpy.ok():
            rclpy.shutdown()

    def _on_ws_event(self, msg):
        if msg.type == "cancel":
            self.exit_gracefully()
        if msg.type == "subs" and msg.count == 0:
            uri = f"ros.service.introspection.{self._service_name.replace('/', '_')}"
            if getattr(msg, "uri", None) == uri:
                Log.info(f"No subscriptions for '{self._service_name}' introspection -> exit!")
                self.exit_gracefully()

    def _subscribe_to_service_event_topic(self):
        topic = f"{self._service_name}/_service_event"

        for _ in range(50):
            topics = dict(self.ros_node.get_topic_names_and_types())
            if topic in topics:
                type_str = topics[topic][0]
                self._create_subscription(topic, type_str)
                return
            time.sleep(0.1)

        Log.warn(
            f"No introspection topic found for '{self._service_name}'. "
            "Is service introspection enabled (METADATA/CONTENTS)?"
        )

    def _create_subscription(self, topic: str, type_str: str):
        try:
            msg_class = get_message(type_str)
        except Exception as e:
            Log.error(f"Could not resolve type '{type_str}' for '{topic}': {e}")
            return

        self._subscription = self.ros_node.create_subscription(msg_class, topic, self._on_service_event, 10)
        Log.info(f"subscribed introspection topic '{topic}' [{type_str}]")

    def _guid_arr_to_str(self, gid: list[int]) -> str:
        return ".".join(f"{c:02X}" for c in gid)

    def _on_service_event(self, msg):
        try:
            info = msg.info
            event_type = EVENT_TYPE_MAP.get(info.event_type, f"unknown({info.event_type})")
            _gid = getattr(info, "client_gid", None)
            client_gid = self._guid_arr_to_str(_gid) if _gid is not None else []
            seq = int(getattr(info, "sequence_number", 0))

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

            event = ServiceIntrospectionEvent(
                service_name=self._service_name,
                event_type=event_type,
                sequence_number=seq,
                client_gid=client_gid,
                data=payload,
                timestamp=time.time(),
            )
            self.wsClient.publish(
                f"ros.service.introspection.{self._service_name.replace('/', '_')}",
                json.dumps(event, cls=SelfEncoder),
            )
        except Exception as e:
            Log.error(f"Error handling service event: {e}\n{traceback.format_exc()}")

    def _init_arg_parser(self) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser()
        parser.add_argument("--ws_port", nargs="?", type=int, required=True)
        parser.add_argument("-s", "--service_name", nargs="?", required=True)
        parser.add_argument("-t", "--service_type", nargs="?", required=True)
        return parser

    def spin(self):
        if self._on_shutdown:
            return
        try:
            rclpy.spin(self.ros_node)
        except KeyboardInterrupt:
            self.exit_gracefully()
        except Exception:
            self.exit_gracefully()


def main():
    launcher = RosServiceIntrospectionLauncher()
    launcher.spin()


if __name__ == "__main__":
    main()
