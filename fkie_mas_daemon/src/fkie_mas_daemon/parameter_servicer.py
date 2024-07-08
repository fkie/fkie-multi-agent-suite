# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from typing import List
import json

from fkie_mas_pylib.interface.runtime_interface import RosParameter
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.parameters.ros1_parameters import ROS1Parameters
from fkie_mas_pylib.websocket.server import WebSocketServer


class ParameterServicer:
    '''
    Interface for ROS1 parameters (using parameter server)
    '''

    def __init__(self, websocket: WebSocketServer, test_env=False) -> None:
        Log.info("Create ROS2 parameter servicer")
        self._handler = ROS1Parameters()
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

    def setParameter(self, parameter: RosParameter):
        '''
        Set the value of a parameter
        '''
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
