#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rcl_interfaces.msg import Log
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class ResetItem:
    def __init__(self, time, msg, color, level):
        self.time = time
        self.msg = msg
        self.color = color
        self.level = level


class RosoutToDiag(Node):

    def __init__(self):
        super().__init__('rosout_to_diag', namespace="mas")

        # Parameter
        self.declare_parameter('duration_reset', 1.0)
        duration_reset = self.get_parameter('duration_reset').value
        self.get_logger().info(f"param duration_reset: {duration_reset}")

        self.declare_parameter('whitelist_nodes', [])
        wl_nodes = self.get_parameter('whitelist_nodes').get_parameter_value().string_array_value
        self.whitelist_nodes = set(wl_nodes)
        self.get_logger().info(f"param whitelist_nodes: {self.whitelist_nodes}")

        self.declare_parameter('blacklist_nodes', [])
        bl_nodes = self.get_parameter('blacklist_nodes').get_parameter_value().string_array_value
        self.blacklist_nodes = set(bl_nodes)
        self.get_logger().info(f"param blacklist_nodes: {self.blacklist_nodes}")

        self.declare_parameter('whitelist_words', [])
        wl_words = self.get_parameter('whitelist_words').get_parameter_value().string_array_value
        self.whitelist_words = set(wl_words)
        self.get_logger().info(f"param whitelist_words: {self.whitelist_words}")

        self.declare_parameter('blacklist_words', [])
        bl_words = self.get_parameter('blacklist_words').get_parameter_value().string_array_value
        self.blacklist_words = set(bl_words)
        self.get_logger().info(f"param blacklist_words: {self.blacklist_words}")

        # Reset-Logik
        self.reset_duration = Duration(seconds=duration_reset)
        self.reset_queue = {}

        # QoS Profile
        qos_transient = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        qos_volatile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publisher & Subscriber
        self.pub_diag = self.create_publisher(DiagnosticArray, '/mas/diagnostics', qos_volatile)
        self.sub_rosout = self.create_subscription(Log, '/rosout', self.callback_log, qos_profile=qos_transient)

        # Timer für Reset
        self.create_timer(0.1, self.timer_reset)

        self.get_logger().info("rosout_to_diag initialized")

    def callback_log(self, msg: Log):
        # Nur WARN, ERROR, FATAL
        if msg.level not in (Log.WARN, Log.ERROR, Log.FATAL):
            return

        # Node-Filter
        if self.whitelist_nodes and msg.name not in self.whitelist_nodes:
            return
        if self.blacklist_nodes and msg.name in self.blacklist_nodes:
            return

        # Word-Filter
        if self.whitelist_words:
            if not any(w in msg.msg for w in self.whitelist_words):
                return
        if any(b in msg.msg for b in self.blacklist_words):
            return

        # Diagnostic-Message aufbauen
        da = DiagnosticArray()
        da.header.stamp = msg.stamp
        status = DiagnosticStatus()
        status.values.append(KeyValue(key='color', value=''))
        status.name = '/' + msg.name
        status.message = msg.msg

        if msg.level == Log.WARN:
            status.level = DiagnosticStatus.WARN
            status.values[0].value = "rgba(255,128,0,1.0)"
            future_color = "rgba(200,100,0,1.0)"
        elif msg.level == Log.ERROR:
            status.level = DiagnosticStatus.ERROR
            status.values[0].value = "rgba(255,0,0,1.0)"
            future_color = "rgba(170,0,0,1.0)"
        else:  # Log.FATAL
            status.level = DiagnosticStatus.ERROR
            status.values[0].value = "rgba(255,0,255,1.0)"
            future_color = "rgba(170,0,170,1.0)"

        da.status.append(status)
        self.pub_diag.publish(da)

        # Reset-Eintrag
        reset_time = self.get_clock().now() + self.reset_duration
        self.reset_queue[status.name] = ResetItem(reset_time,
                                                  status.message,
                                                  future_color,
                                                  status.level)

        self.get_logger().debug(f"{msg.name}: {status.values[0].value} '{msg.msg}'")

    def timer_reset(self):
        now = self.get_clock().now()
        to_delete = []
        for name, item in self.reset_queue.items():
            if now >= item.time:
                da = DiagnosticArray()
                da.header.stamp = now
                status = DiagnosticStatus()
                status.name = name
                status.message = item.msg
                status.level = item.level
                status.values.append(KeyValue(key='color', value=item.color))
                da.status.append(status)
                self.pub_diag.publish(da)
                to_delete.append(name)
        for name in to_delete:
            del self.reset_queue[name]


def main(args=None):
    rclpy.init(args=args)
    node = RosoutToDiag()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
