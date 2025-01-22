from typing import List
from rclpy.node import Node

from fkie_mas_pylib.parameters.ros2_parameter_interface import ParameterInterface
from fkie_mas_pylib.interface.runtime_interface import RosParameter
from fkie_mas_pylib.logging.logging import Log


class ROS2Parameters:
    def __init__(self, node: Node) -> None:
        self.interface = ParameterInterface(node)

    def getParameterList(self) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types
        '''
        param_list: List[RosParameter] = []
        param_list = self.interface.list()
        return param_list

    def getNodeParameters(self, nodes: List[str]) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        param_list: List[RosParameter] = []
        param_list = self.interface.list(nodes)
        return param_list

    def setParameter(self, parameter: RosParameter) -> bool:
        '''
        Set the value of a parameter
        '''
        return self.interface.set(parameter)

    def deleteParameter(self, parameters: List[str], nodeName: str) -> bool:
        '''
        Delete a list of parameter
        '''
        overall_result = True
        for parameter in parameters:
            if not self.interface.delete(parameter, nodeName):
                overall_result = False
        return overall_result
