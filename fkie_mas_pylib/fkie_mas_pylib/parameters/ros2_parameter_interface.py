import json
import time
import yaml
from typing import List
from typing import Tuple
from typing import Union
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import DescribeParameters
from ros2node.api import get_node_names
from fkie_mas_pylib.service.future import WaitFuture
from fkie_mas_pylib.service.future import create_service_future
from fkie_mas_pylib.service.future import wait_until_futures_done
from ros2service.api import get_service_names
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue

from fkie_mas_pylib.interface.runtime_interface import RosParameter
from fkie_mas_pylib.interface.runtime_interface import RosParameterRange
from fkie_mas_pylib.logging.logging import Log


class ParameterInterface:
    """ROS2 Parameter interface"""

    def __init__(self, global_node: Node) -> None:
        self.include_hidden_nodes = True  # Consider hidden nodes as well
        self.param_prefixes = []  # Only list parameters with the provided prefixes
        self.global_node = global_node

    def list(self, nodes: List[str] = None) -> Tuple[List[RosParameter], List[str]]:
        param_list: List[RosParameter] = []
        errors = []

        # get nodes and services
        node_names = get_node_names(
            node=self.global_node, include_hidden_nodes=self.include_hidden_nodes)

        service_names = get_service_names(
            node=self.global_node, include_hidden_services=self.include_hidden_nodes)

        wait_futures: List[WaitFuture] = []
        # create clients for nodes which have the service
        for node_name in node_names:
            if nodes is not None and node_name.full_name not in nodes:
                continue
            service_name = f'{node_name.full_name}/list_parameters'
            if service_name in service_names:
                Log.debug(f"{self.__class__.__name__}:  list_parameters '{service_name}'")
                ready = create_service_future(self.global_node, wait_futures, "list_parameters", node_name.full_name, service_name,
                                              ListParameters, ListParameters.Request())
                if not ready:
                    Log.debug(f"{self.__class__.__name__}:    service '{service_name}' is not ready, skip")
                    errors.append(f"service '{service_name}' is not ready, skip")

        # wait until all clients have been called
        wait_until_futures_done(wait_futures)

        # read list parameter responses
        node_parameters = {}
        for wait_future in wait_futures:
            if wait_future.finished:
                try:
                    response = wait_future.future.result()
                    if response:
                        node_parameters[wait_future.node_name] = sorted(response.result.names)
                except Exception as exception:
                    Log.warn(
                        f"{self.__class__.__name__}:-> failed to list parameter calling '{wait_future.service_name}': '{exception}'")
            else:
                Log.warn(f"{self.__class__.__name__}:-> Timeout while calling '{wait_future.service_name}'")
                errors.append(f"Timeout while calling '{wait_future.service_name}'")
            wait_future.client.destroy()

        # get parameter values
        wait_futures = []
        for node_name, parameters in node_parameters.items():
            service_name = f"{node_name}/get_parameters"
            Log.debug(f"{self.__class__.__name__}:  get parameters '{service_name}'")
            request = GetParameters.Request()
            request.names = parameters
            ready = create_service_future(self.global_node, wait_futures, "get_parameters", node_name, service_name,
                                          GetParameters, request)
            if not ready:
                Log.debug(f"{self.__class__.__name__}:    service '{service_name}' is not ready, skip")
                errors.append(f"service '{service_name}' is not ready, skip")

        # wait until all clients have been called
        wait_until_futures_done(wait_futures)

        # read get parameter responses
        for wait_future in wait_futures:
            if wait_future.finished:
                try:
                    response = wait_future.future.result()
                    if response:
                        for (index, parameter) in enumerate(response.values):
                            param_name = f'{node_parameters[wait_future.node_name][index]}'
                            param_list.append(
                                RosParameter(wait_future.node_name, param_name, self._get_value(parameter), self._get_type(parameter)))
                except Exception as exception:
                    Log.warn(
                        f"{self.__class__.__name__}:-> failed to get parameter calling '{wait_future.service_name}': '{exception}'")
            else:
                Log.warn(f"{self.__class__.__name__}:-> Timeout while calling '{wait_future.service_name}'")
                errors.append(f"Timeout while calling '{wait_future.service_name}'")
            wait_future.client.destroy()

        # get parameter descriptions
        wait_futures = []
        for node_name, parameters in node_parameters.items():
            service_name = f"{node_name}/describe_parameters"
            Log.debug(f"{self.__class__.__name__}:  describe parameters '{service_name}'")
            request = DescribeParameters.Request()
            request.names = parameters
            ready = create_service_future(self.global_node, wait_futures, "describe_parameters", node_name, service_name,
                                          DescribeParameters, request)
            if not ready:
                Log.debug(f"{self.__class__.__name__}:    service '{service_name}' is not ready, skip")
                errors.append(f"service '{service_name}' is not ready, skip")

        # wait until all clients have been called
        wait_until_futures_done(wait_futures)

        # read get parameter responses
        for wait_future in wait_futures:
            if wait_future.finished:
                try:
                    response = wait_future.future.result()
                    if response:
                        for (index, descriptor) in enumerate(response.descriptors):
                            param_name = f'{node_parameters[wait_future.node_name][index]}'
                            parameter = param_list[index]
                            if parameter.name != descriptor.name:
                                # should not happen
                                Log.warn(
                                    f"{self.__class__.__name__}:-> descriptor '{descriptor.name}' assigned to wrong parameter: '{parameter.name}'")
                            parameter.readonly = descriptor.read_only
                            parameter.description = descriptor.description
                            parameter.additional_constraints = descriptor.additional_constraints
                            for fpr in descriptor.floating_point_range:
                                parameter.floating_point_range.append(
                                    RosParameterRange(fpr.from_value, fpr.to_value, fpr.step))
                            for ir in descriptor.integer_range:
                                parameter.integer_range.append(
                                    RosParameterRange(ir.from_value, ir.to_value, ir.step))
                except Exception as exception:
                    Log.warn(
                        f"{self.__class__.__name__}:-> failed to get parameter calling '{wait_future.service_name}': '{exception}'")
                    errors.append(f"failed to get parameter calling '{wait_future.service_name}': '{exception}'")
            else:
                Log.warn(f"{self.__class__.__name__}:-> Timeout while calling '{wait_future.service_name}'")
                errors.append(f"Timeout while calling '{wait_future.service_name}'")
            wait_future.client.destroy()

        return param_list, errors

    def get(self, _parameter: RosParameter) -> Union[RosParameter, None]:
        node_name = self._get_node_name(_parameter)
        if node_name is None:
            return False

        parameter_name = _parameter.name.replace(f'{node_name}/', '')
        response = self.call_get_parameters(
            node=self.global_node, node_name=node_name, parameters=[parameter_name])

        # output response
        if response is None or len(response.values) == 0:
            return None

        for parameter in response.values:
            return RosParameter(node_name, parameter_name, self._get_value(parameter), self._get_type(parameter))

        raise Exception(f"Get parameter failed: {parameter_name}")

    def set(self, _parameter: RosParameter) -> bool:
        node_name = self._get_node_name(_parameter)
        if node_name is None:
            return False

        parameter = Parameter()
        parameter.name = _parameter.name.replace(f'{node_name}/', '')
        parameter.value = self._get_parameter_value(_parameter)

        response = self.call_set_parameters(
            node=self.global_node, node_name=node_name, parameters=[parameter])

        # output response
        if response is None or len(response.results) == 0:
            return False

        result = response.results[0]
        if result.successful:
            return True

        raise Exception(f"Setting parameter failed: {parameter}; reason: {result.reason}")

    def delete(self, parameter_name: str, node: str):
        node_name = node
        if not node_name or len(node_name) == 0:
            node_name = self._get_node_name(parameter_name)
        if node_name is None:
            raise Exception(f'Deleting parameter failed: ', parameter, "Node name not found")

        parameter = Parameter()
        parameter.name = parameter_name.replace(f'{node_name}/', '')
        value = ParameterValue()
        value.type = ParameterType.PARAMETER_NOT_SET
        parameter.value = value

        response = self.call_set_parameters(
            node=self.global_node, node_name=node_name, parameters=[parameter])

        # output response
        if response is None or len(response.results) == 0:
            raise Exception(f'Deleting parameter failed: ', parameter, "Empty result")

        result = response.results[0]
        if result.successful:
            return True

        raise Exception(f"Deleting parameter failed: {parameter}; reason: {result.reason}")

    def _get_value(self, parameter_value):
        """Get the value from a ParameterValue."""
        if parameter_value.type == ParameterType.PARAMETER_BOOL:
            value = parameter_value.bool_value
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
            value = parameter_value.integer_value
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
            value = parameter_value.double_value
        elif parameter_value.type == ParameterType.PARAMETER_STRING:
            value = parameter_value.string_value
        elif parameter_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            value = list(parameter_value.byte_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            value = list(parameter_value.bool_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            value = list(parameter_value.integer_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            value = list(parameter_value.double_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            value = list(parameter_value.string_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_NOT_SET:
            value = None
        else:
            value = None

        return value

    def _get_type(self, parameter_value):
        """Get the value from a ParameterValue."""
        if parameter_value.type == ParameterType.PARAMETER_BOOL:
            return "bool"
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
            return "int"
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
            return "float"
        elif parameter_value.type == ParameterType.PARAMETER_STRING:
            return "str"
        elif parameter_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return "byte[]"
        elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return "bool[]"
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return "int[]"
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return "float[]"
        elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return "str[]"
        elif parameter_value.type == ParameterType.PARAMETER_NOT_SET:
            value = None
        else:
            value = None

        return value

    def _get_parameter_value(self, parameter: RosParameter) -> ParameterValue:
        value = ParameterValue()

        if parameter.get_type() == 'bool':
            value.type = ParameterType.PARAMETER_BOOL
            value.bool_value = parameter.typed_value()
        elif parameter.get_type() == 'int':
            value.type = ParameterType.PARAMETER_INTEGER
            value.integer_value = parameter.typed_value()
        elif parameter.get_type() == 'float':
            value.type = ParameterType.PARAMETER_DOUBLE
            value.double_value = parameter.typed_value()
        elif parameter.get_type().endswith("[]"):
            if parameter.get_type() == 'bool[]':
                value.type = ParameterType.PARAMETER_BOOL_ARRAY
                value.bool_array_value = parameter.typed_value()
            elif parameter.get_type() == 'int[]':
                value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                value.integer_array_value = parameter.typed_value()
            elif parameter.get_type() == 'float[]':
                value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                value.double_array_value = parameter.typed_value()
            elif parameter.get_type() == 'str[]':
                value.type = ParameterType.PARAMETER_STRING_ARRAY
                value.string_array_value = parameter.typed_value()
            else:
                value.type = ParameterType.PARAMETER_STRING
                value.string_value = parameter.typed_value()
        else:
            value.type = ParameterType.PARAMETER_STRING
            value.string_value = parameter.typed_value()
        return value

    def _get_node_name(self, parameter_name: Union[str, RosParameter]) -> str:
        name = parameter_name
        if isinstance(parameter_name, RosParameter):
            if hasattr(parameter_name, "node"):
                return parameter_name.node
            else:
                name = parameter_name.name
        # get node name and parameter name
        p_split = name.split("/")
        if len(p_split) == 0:
            return None

        # TODO: Fix qos_overrides parameters

        param_name = p_split.pop()
        node_name = name.replace(f'/{param_name}', "")
        return node_name

    def call_set_parameters(self, *, node, node_name, parameters):
        # create client
        wait_futures: List[WaitFuture] = []
        # create clients for nodes which have the service
        service_name = f'{node_name}/set_parameters'
        Log.debug(f"{self.__class__.__name__}:  set_parameters '{service_name}'")
        request = SetParameters.Request()
        request.parameters = parameters
        ready = create_service_future(node, wait_futures, "set_parameters", node_name, service_name,
                                      SetParameters, request)
        if not ready:
            Log.debug(f"{self.__class__.__name__}:    service '{service_name}' is not ready, skip")

        # wait until all clients have been called
        wait_until_futures_done(wait_futures)

        # read set parameter responses
        for wait_future in wait_futures:
            if wait_future.finished:
                response = wait_future.future.result()
                if response:
                    return response
            else:
                wait_future.client.destroy()
                # raise Exception(f"Timeout while calling '{wait_future.service_name}'")
        return None

    def call_get_parameters(self, *, node, node_name: str, parameters: List[str]):
        # create client
        wait_futures: List[WaitFuture] = []
        # create clients for nodes which have the service
        service_name = f'{node_name}/get_parameters'
        Log.debug(f"{self.__class__.__name__}:  get_parameters '{service_name}'")
        request = GetParameters.Request()
        request.names = parameters
        ready = create_service_future(node, wait_futures, "get_parameter", node_name, service_name,
                                      GetParameters, request)
        if not ready:
            Log.debug(f"{self.__class__.__name__}:    service '{service_name}' is not ready, skip")

        # wait until all clients have been called
        wait_until_futures_done(wait_futures)

        # read set parameter responses
        for wait_future in wait_futures:
            if wait_future.finished:
                response = wait_future.future.result()
                if response:
                    return response
            else:
                wait_future.client.destroy()
                # raise Exception(f"Timeout while calling '{wait_future.service_name}'")
        return None
