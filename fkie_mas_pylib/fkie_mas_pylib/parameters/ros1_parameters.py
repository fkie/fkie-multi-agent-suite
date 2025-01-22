from typing import List
import rospy

from fkie_mas_pylib.interface.runtime_interface import RosParameter
from fkie_mas_pylib.logging.logging import Log


class ROS1Parameters:

    def getParameterList(self) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types
        '''
        param_list: List[RosParameter] = []
        param_name_list = []

        param_name_list = rospy.get_param_names()

        # get parameter values
        for param_name in param_name_list:
            param_value = rospy.get_param(param_name)
            param_list.append(RosParameter("", param_name, param_value))
        return param_list

    def getNodeParameters(self, nodes: List[str]) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        param_list = []
        param_name_list = []

        param_name_list = rospy.get_param_names()

        for node_name in nodes:
            for param_name in param_name_list:
                # discard parameters that does not belong to the node
                if f'{node_name}/' not in param_name:
                    continue
                param_value = rospy.get_param(param_name)
                param_list.append(RosParameter("", param_name, param_value))
        return param_list

    def setParameter(self, parameter: RosParameter) -> bool:
        '''
        Set the value of a parameter
        '''
        rospy.set_param(f"{parameter.name}", RosParameter(parameter.node, parameter.name, parameter.value, parameter.type).typed_value())
        return True

    def deleteParameter(self, parameters: List[str]) -> bool:
        '''
        Delete a list of parameter
        '''
        for parameter in parameters:
            rospy.delete_param(parameter)
        return True
