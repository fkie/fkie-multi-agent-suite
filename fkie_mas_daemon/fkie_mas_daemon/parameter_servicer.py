from typing import List
import json
import asyncio

from fkie_mas_pylib.interface.runtime_interface import RosParameter
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.parameters.ros2_parameters import ROS2Parameters
from fkie_mas_pylib.websocket.server import WebSocketServer
import fkie_mas_daemon as nmd


class ParameterServicer:
    '''
    Interface for ROS2 parameters
    '''

    def __init__(self, websocket: WebSocketServer, test_env=False) -> None:
        Log.info("Create ROS2 parameter servicer")
        self._handler = ROS2Parameters(nmd.ros_node)
        websocket.register("ros.parameters.get_list", self.getParameterList)
        websocket.register("ros.parameters.get_node_parameters", self.getNodeParameters)
        websocket.register("ros.parameters.has_parameter", self.hasParameter)
        websocket.register("ros.parameters.set_parameter", self.setParameter)
        websocket.register("ros.parameters.delete_parameters", self.deleteParameters)

    def stop(self):
        pass

    def getParameterList(self):
        '''
        Return a list with all registered parameters values and types
        '''
        p_list = []
        Log.info(
            'ros.parameters.get_list: Getting parameters for all nodes')
        p_list = self._handler.getParameterList()
        return json.dumps(p_list, cls=SelfEncoder)

    def getNodeParameters(self, nodes: List[str]):
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        p_list = []
        Log.info(
            f'ros.parameters.get_node_parameters: Getting parameters for nodes: [{nodes}] ')
        p_list = self._handler.getNodeParameters(nodes)
        return json.dumps(p_list, cls=SelfEncoder)

    def hasParameter(self, parameter_name: str):
        '''
        Check if a parameter exists
        '''
        result = False
        Log.info(
            f'ros.parameters.has_parameter: Checking parameter [{parameter_name}] ')
        result = self._handler.hasParameter(parameter_name)
        return json.dumps(result, cls=SelfEncoder)

    def setParameter(self, _parameter: RosParameter):
        '''
        Set the value of a parameter
        '''
        parameter = _parameter
        Log.info(
            f'ros.parameters.set_parameter: [{parameter.name}] to {parameter.value}')
        result = None
        result = self._handler.setParameter(parameter)
        return json.dumps(result, cls=SelfEncoder)

    def deleteParameters(self, parameters: List[str]):
        '''
        Delete a list of parameters
        '''
        result = False
        Log.info(
            f'ros.parameters.delete_parameters: Removing [{parameters}] ')
        result = self._handler.deleteParameter(parameters)
        return json.dumps(result, cls=SelfEncoder)
