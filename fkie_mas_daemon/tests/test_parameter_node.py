#! /usr/bin/env python3

"""
Create a ROS2 node with different parameters to test the MAS GUI
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rclpy.parameter import Parameter
from std_msgs.msg import Int32
from std_msgs.msg import String
from typing import List


class TestParameter(Node):

    def __init__(self):
        super().__init__('test_parameter_node')

        # Describe parameters
        integer_descriptor = ParameterDescriptor(
            description='This is a test parameter with integer range', integer_range=[IntegerRange(from_value=-2, to_value=30, step=2)])
        double_descriptor = ParameterDescriptor(
            description='This is a test parameter with double range', floating_point_range=[FloatingPointRange(from_value=-1.5, to_value=25.5, step=0.5)])
        double_wo_step_descriptor = ParameterDescriptor(
            description='This is a test parameter with double range, w/o step and no.step namespace', floating_point_range=[FloatingPointRange(from_value=-1.5, to_value=25.5)])
        string_descriptor = ParameterDescriptor(description='String parameter')

        # Declare parameters
        self.declare_parameter('integer', 4, integer_descriptor)
        self.declare_parameter('double', 2.5, double_descriptor)
        self.declare_parameter('no.step.double', 2.5, double_wo_step_descriptor)
        self.declare_parameter('string', 'Robot Name', string_descriptor)
        self.declare_parameter('single.string', 'With Namespace')
        self.declare_parameter('array.bool', [True, False])
        self.declare_parameter('array.double', [1.1, 2.2, 3.3])
        self.declare_parameter('array.integer', [1, 2, 3])
        self.declare_parameter('array.string', ["one", "two", "with space"])
        self.declare_parameter('error.while.set.bool', True)

        # Register a callback function that will be called whenever there is an attempt to
        # change one or more parameters of the node.
        self.add_on_set_parameters_callback(self.parameter_change_callback)

        # create test subscriptions
        self.sub_string = self.create_subscription(String, "one/two/test/sub", self.on_string, 10)
        self.sub_int = self.create_subscription(Int32, "two/test/int", self.on_int, 10)

    def parameter_change_callback(self, params: List[Parameter]) -> SetParametersResult:
        result = SetParametersResult()
        result.successful = True
        for param in params:
            # Check the parameter's name and type
            if param.name == "double" and param.type_ == Parameter.Type.DOUBLE:
                self.get_logger().info("Parameter integer has changed. The new value is: %f" % param.value)
            if param.name == "string" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("Parameter string has changed. The new value is: %s" % param.value)
            if param.name == "array.bool" and param.type_ == Parameter.Type.BOOL_ARRAY:
                self.get_logger().info("Parameter array.bool has changed. The new value is: %s" % param.value)
            if param.name == "array.double" and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                self.get_logger().info("Parameter array.double has changed. The new value is: %s" % param.value)
            if param.name == "array.integer" and param.type_ == Parameter.Type.INTEGER_ARRAY:
                self.get_logger().info("Parameter array.integer has changed. The new value is: %s" % param.value)
            if param.name == "error.while.set.bool":
                result.successful = False
            print(f"param: {param.name} [{param.type_}] {param.value} - {result.successful}")
        return result

    def on_string(self, msg: String):
        print(f"received string message: {msg}")

    def on_int(self, msg: String):
        print(f"received int32 message: {msg}")

def main(args=None):
    rclpy.init(args=args)
    test_parameter_node = TestParameter()
    rclpy.spin(test_parameter_node)
    test_parameter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
