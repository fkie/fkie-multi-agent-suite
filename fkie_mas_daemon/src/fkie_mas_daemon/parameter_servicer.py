# The MIT License (MIT)

# Copyright (c) 2014-2024 Fraunhofer FKIE, Alexander Tiderko

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from typing import List
import json
import asyncio
from autobahn import wamp
from types import SimpleNamespace

from fkie_mas_pylib.crossbar.runtime_interface import RosParameter
from fkie_mas_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_mas_pylib.crossbar.base_session import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.parameters.ros1_parameters import ROS1Parameters


class ParameterServicer(CrossbarBaseSession):
    '''
    Interface for ROS1 parameters (using parameter server)
    '''

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911, test_env=False) -> None:
        Log.info("Create ROS2 parameter servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env=test_env)
        self._handler = ROS1Parameters()

    @wamp.register('ros.parameters.get_list')
    def getParameterList(self):
        '''
        Return a list with all registered parameters values and types
        '''
        p_list = []
        Log.info(
            'ros.parameters.get_list: Getting parameters for all nodes')
        p_list = self._handler.getParameterList()
        return json.dumps(p_list, cls=SelfEncoder)

    @wamp.register('ros.parameters.get_node_parameters')
    def getNodeParameters(self, nodes: List[str]):
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        p_list = []
        Log.info(
            f'ros.parameters.get_node_parameters: Getting parameters for nodes: [{nodes}] ')
        p_list = self._handler.getNodeParameters(nodes)
        return json.dumps(p_list, cls=SelfEncoder)

    @wamp.register('ros.parameters.has_parameter')
    def hasParameter(self, parameter_name: str):
        '''
        Check if a parameter exists
        '''
        result = False
        Log.info(
            f'ros.parameters.has_parameter: Checking parameter [{parameter_name}] ')
        result = self._handler.hasParameter(parameter_name)
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.parameters.set_parameter')
    def setParameter(self, _parameter: RosParameter):
        '''
        Set the value of a parameter
        '''
        parameter = json.loads(json.dumps(_parameter),
                               object_hook=lambda d: SimpleNamespace(**d))
        Log.info(
            f'ros.parameters.set_parameter: [{parameter.name}] to {parameter.value}')
        result = None
        result = self._handler.setParameter(parameter)
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.parameters.delete_parameters')
    def deleteParameters(self, parameters: List[str]):
        '''
        Delete a list of parameters
        '''
        result = False
        Log.info(
            f'ros.parameters.delete_parameters: Removing [{parameters}] ')
        result = self._handler.deleteParameter(parameters)
        return json.dumps(result, cls=SelfEncoder)
