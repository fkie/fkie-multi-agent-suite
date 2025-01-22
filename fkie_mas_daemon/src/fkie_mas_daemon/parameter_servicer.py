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
        websocket.register("ros.parameters.set_parameter", self.setParameter)
        websocket.register("ros.parameters.delete_parameters", self.deleteParameters)
    
    def stop(self):
        pass

    def getParameterList(self):
        '''
        Return a list with all registered parameters values and types
        '''
        p_list = []
        try:
            Log.info('ros.parameters.get_list: Getting parameters for all nodes')
            p_list = self._handler.getParameterList()
        except Exception:
            import traceback
            print(traceback.format_exc())
        return json.dumps(p_list, cls=SelfEncoder)

    def getNodeParameters(self, nodes: List[str]):
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        p_list = []
        try:
            Log.info(f'ros.parameters.get_node_parameters: Getting parameters for nodes: [{nodes}] ')
            p_list = self._handler.getNodeParameters(nodes)
        except Exception:
            import traceback
            print(traceback.format_exc())
        return json.dumps(p_list, cls=SelfEncoder)

    def setParameter(self, paramName: str, paramType: str, paramValue: str, nodeName: str):
        '''
        Set the value of a parameter
        '''
        result = None
        try:
            Log.info(f'ros.parameters.set_parameter: [{paramName}] to {paramValue}')
            _parameter = RosParameter(nodeName, paramName, paramValue, paramType)
            res = self._handler.setParameter(_parameter)
            result = {"result": res, "message" : ""}
        except Exception as error:
            import traceback
            print(traceback.format_exc())
            result = {"result": False, "message" : f"{error}"}
        return json.dumps(result, cls=SelfEncoder)

    def deleteParameters(self, parameters: List[str], nodeName: str):
        '''
        Delete a list of parameters
        '''
        result = None
        try:
            Log.info(f'ros.parameters.delete_parameters: Removing [{parameters}] ')
            res = self._handler.deleteParameter(parameters)
            result = {"result": res, "message" : ""}
        except Exception as error:
            import traceback
            print(traceback.format_exc())
            result = {"result": False, "message" : f"{error}"}
        return json.dumps(result, cls=SelfEncoder)
