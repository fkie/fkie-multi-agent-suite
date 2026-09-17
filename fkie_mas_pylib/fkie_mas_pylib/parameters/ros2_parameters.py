from rclpy.node import Node

from fkie_mas_pylib.interface.runtime_interface import RosParameter
from fkie_mas_pylib.parameters.ros2_parameter_interface import ParameterInterface


class ROS2Parameters:
    def __init__(self, node: Node) -> None:
        self.interface = ParameterInterface(node)

    def getParameterList(self) -> list[RosParameter]:
        """
        Return a list with all registered parameters values and types
        """
        param_list: list[RosParameter] = []
        param_list = self.interface.list()
        return param_list

    def getNodeParameters(self, nodes: list[str]) -> tuple[list[RosParameter], list[str]]:
        """
        Return a list with all registered parameters values and types for a given Node
        """
        param_list: list[RosParameter] = []
        param_list, errors = self.interface.list(nodes)
        return param_list, errors

    def setParameter(self, parameter: RosParameter) -> RosParameter | None:
        """
        Set the value of a parameter
        """
        if self.interface.set(parameter):
            return self.interface.get(parameter)
        return None

    def deleteParameter(self, parameters: list[str], nodeName: str) -> bool:
        """
        Delete a list of parameter
        """
        overall_result = True
        for parameter in parameters:
            if not self.interface.delete(parameter, nodeName):
                overall_result = False
        return overall_result
