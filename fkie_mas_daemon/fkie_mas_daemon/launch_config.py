# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from pathlib import Path

from typing import Dict
from typing import List
from typing import Set
from typing import Text
from typing import Tuple
from typing import Union

import ruamel.yaml
from xml.dom.minidom import parse  # , parseString
import os
import re
import shlex
import sys
import threading
import time

from ament_index_python.packages import PackageNotFoundError
import launch
from launch.launch_context import LaunchContext
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.frontend.parser import Parser
from launch.substitutions.substitution_failure import SubstitutionFailure
import launch.utilities
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.utilities.evaluate_parameters import evaluate_parameters
from launch_ros.utilities import to_parameters_list
from launch.utilities import perform_substitutions
import launch_ros
import composition_interfaces.srv
from launch_ros.parameter_descriptions import ParameterFile

from launch_ros.substitutions.find_package import FindPackage
from launch_ros.utilities import make_namespace_absolute
from launch_ros.utilities import prefix_namespace
from launch_ros.utilities.normalize_parameters import normalize_parameter_dict
from launch_ros.substitutions.executable_in_package import ExecutableInPackage

from fkie_mas_pylib.interface.runtime_interface import RosParameter
from fkie_mas_pylib.interface.launch_interface import LaunchArgument
from fkie_mas_pylib.interface.launch_interface import LaunchIncludedFile
from fkie_mas_pylib.interface.launch_interface import LaunchNodeInfo
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib import names
from fkie_mas_pylib import ros_pkg
from fkie_mas_pylib.defines import RESPAWN_SCRIPT
from fkie_mas_pylib.defines import SEP
from fkie_mas_pylib.system import exceptions
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.system.supervised_popen import SupervisedPopen

import fkie_mas_daemon as nmd

PRINT_DEBUG_LOAD = False


class LaunchConfigException(Exception):
    pass


def perform_to_string(context: launch.LaunchContext, value: Union[List[List], List[launch.Substitution], str, None]) -> Union[str, None]:
    result = ''
    if isinstance(value, str):
        result = value
    elif isinstance(value, List) and len(value) > 0:
        for val in value:
            sep = ' '
            if isinstance(val, List):
                item = ""
                try:
                    item = perform_substitutions(context, val)
                except (SubstitutionFailure, LookupError) as err:
                    # if executable is not found we replace it by "ros2 run" command to visualize the error in the MAS gui
                    if isinstance(val[0], ExecutableInPackage):
                        executable = perform_substitutions(context, val[0].executable)
                        package = perform_substitutions(context, val[0].package)
                        item = f"ros2 run {package} {executable}"
                    else:
                        raise err
                except Exception as err:
                    import traceback
                    print(traceback.format_exc())
                    raise LaunchConfigException(err)
                # TODO: should we fix command lines with {data: xyz}
                # if ' ' in item and '{' in item:
                #     item = f"'{item}'"
                result += item + sep
            else:
                result += perform_to_string(context, val)
    elif value is not None:
        try:
            if isinstance(value, tuple):
                for tuple_item in value:
                    result += perform_substitutions(context, [tuple_item])
            else:
                item = perform_substitutions(context, [value])
                result = perform_substitutions(context, [value])
        except (SubstitutionFailure, LookupError) as err:
            # if executable is not found we replace it by "ros2 run" command to visualize the error in the MAS gui
            if isinstance(value, ExecutableInPackage):
                executable = perform_substitutions(context, value.executable)
                package = perform_substitutions(context, value.package)
                result = f"ros2 run {package} {executable}"
            else:
                raise err
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            raise LaunchConfigException(err)
    else:
        result = None
    return result


def perform_to_tuple_list(context: launch.LaunchContext, value: Union[List[Tuple[List[launch.Substitution], List[launch.Substitution]]], None]) -> Union[List[Tuple[str, str]], None]:
    result = []
    if value is not None:
        for val1, val2 in value:
            result.append((perform_substitutions(context, val1),
                          perform_substitutions(context, val2)))
    else:
        result = None
    return result


class LaunchNodeWrapper(LaunchNodeInfo):

    # _unique_names: Set[str] = set()
    # _remapped_names: Dict[str, Set[str]] = {}

    def __init__(self, entity: launch.actions.ExecuteProcess, launch_description: Union[launch.LaunchDescription, launch.actions.IncludeLaunchDescription], launch_context: launch.LaunchContext, composable_container: str = None, environment: Dict = {}, position_in_file=0) -> None:
        self._entity = entity
        self._launch_description = launch_description
        self._launch_context = launch_context
        self.timer_period = 0
        if isinstance(self._entity, launch_ros.actions.Node):
            # Prepare the ros_specific_arguments list and add it to the context so that the
            # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
            ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
            if self._entity._Node__node_name is not None:
                ros_specific_arguments['name'] = f'__node:={self._entity._Node__expanded_node_name}'
            if self._entity._Node__expanded_node_namespace != '':
                ros_specific_arguments['ns'] = f'__ns:={self._entity._Node__expanded_node_namespace}'

            # Give extensions a chance to prepare for execution
            for extension in self._entity._Node__extensions.values():
                cmd_extension, ros_specific_arguments = extension.prepare_for_execute(
                    self._launch_context,
                    ros_specific_arguments,
                    self
                )
                self._entity._Node.cmd.extend(cmd_extension)

            self._launch_context.extend_locals(
                {'ros_specific_arguments': ros_specific_arguments})

        node_name, unique_name, name_configured = self._get_name()
        LaunchNodeInfo.__init__(
            self, unique_name=unique_name, node_name=node_name, name_configured=name_configured)
        if PRINT_DEBUG_LOAD:
            print("  ***debug LaunchNodeWrapper: name_configured", name_configured)
        self.node_namespace = self._get_namespace()
        self.package_name = self._get_node_package()
        self.executable = self._get_node_executable()
        self.respawn = self._get_respawn()
        self.respawn_delay = self._get_respawn_delay()
        if isinstance(self._launch_description, launch.actions.IncludeLaunchDescription):
            # we use only real path paths!
            self.file_name = os.path.realpath(self._launch_description._get_launch_file())
            self.launch_name = getattr(self._launch_context.locals, 'launch_file_path', None)
            # add launch arguments used to load the included file
            if self._launch_description.launch_arguments:
                self.launch_context_arg = []
            for arg_name, arg_value in self._launch_description.launch_arguments:
                self.launch_context_arg.append(LaunchArgument(perform_to_string(self._launch_context, arg_name),
                                                              perform_to_string(self._launch_context, arg_value)))
        else:
            self.launch_name = getattr(self._launch_description, 'launch_name', '')
            self.launch_context_arg = getattr(self._launch_context.locals, 'launch_arguments', None)
            self.file_name = self.launch_name
        self.composable_container: str = composable_container
        self._parameters_tmp = self._get_parameter_arguments()
        self.parameters = []
        for p in self._parameters_tmp:
            if isinstance(p, tuple) and p[0].startswith("/"):
                if os.path.exists(p[0]):
                    with open(p[0]) as tmp_param_file:
                        try:
                            yaml = ruamel.yaml.YAML(typ='rt')
                            self.parameters.append(RosParameter(node_name, p[0], yaml.load(tmp_param_file)))
                            continue
                        except ruamel.yaml.YAMLError as exc:
                            pass
                self.parameters.append(RosParameter(node_name, p[0], p[1]))
                continue
            elif isinstance(p, dict):
                for key, val in p.items():
                    p_name = None
                    p_val = None
                    if isinstance(key, tuple):
                        p_name = perform_to_string(self._launch_context, key)
                    if isinstance(val, tuple):
                        p_val = perform_to_string(self._launch_context, val)
                    elif hasattr(val, "value"):
                        p_val = val.value
                    if p_name is not None and p_val is not None:
                        self.parameters.append(RosParameter(node_name, p_name, p_val))
                continue
            print(f"ignored new parameter type: {type(p)}: {p}")
            # self.parameters.append(p)

        self.args = self._get_arguments()
        self.cmd = perform_to_string(self._launch_context, getattr(self._entity, 'cmd', None))
        self.cwd = perform_to_string(self._launch_context, getattr(self._entity, 'cwd', None))
        self.env = dict(environment)
        env = perform_to_tuple_list(self._launch_context, getattr(self._entity, 'env', None))
        if env:
            self.env.update(env)
        self.additional_env = perform_to_tuple_list(self._launch_context, getattr(self._entity, 'additional_env', None))
        self.launch_prefix = perform_to_string(self._launch_context, self._get_launch_prefix())
        self._load_node_request = None
        if self.composable_container:
            self._load_node_request = self._create_composed_load_request(self._launch_context)

        #  remap_args: List[Tuple[str, str]] = None,
        #  output: str = '',
        #  output_format: str = '',
        #  sigterm_timeout: str = '',
        #  sigkill_timeout: str = '',
        #  on_exit: List[Any] = [],
        #  required: bool = False,
        #  file_name: str = '',
        #  file_range: Dict[str, Number] = {"startLineNumber": 0,
        #                                   "endLineNumber": 0,
        #                                   "startColumn": 0,
        #                                   "endColumn": 0},
        #  launch_context_arg: str = '',
        #  launch_name: str = ''
        #  composable_container: str = ''
        #  Search the line number of a given node in launch file
        if (self.file_name):
            node_base_name = os.path.basename(node_name)
            name_select_len = len(node_base_name) + 7
            lines_with_node_name = []
            with open(self.file_name, "r") as launch_file:
                current_position = 0
                for line_number, line_text in enumerate(launch_file):
                    # search only after given position
                    if current_position >= position_in_file:
                        start_column = line_text.find(f'name="{node_base_name}"')
                        if start_column < 0:
                            start_column = line_text.find(
                                f"name='{node_base_name}'")
                        if start_column > -1:
                            start_column += 1
                            lines_with_node_name.append(
                                [line_number + 1, line_text, start_column, start_column + name_select_len])
                    current_position += len(line_text) + 1
            line_number = -1
            start_column = 0
            end_column = 0
            line_text = ""
            if len(lines_with_node_name) == 0:
                # no line found. TODO: Report error?
                line_number = 0
            elif len(lines_with_node_name) >= 1:
                line_number = lines_with_node_name[0][0]
                line_text = lines_with_node_name[0][1]
                start_column = lines_with_node_name[0][2]
                end_column = lines_with_node_name[0][3]
            # elif len(lines_with_node_name) > node_occurrence[item.launch_name]:
            #     # More than one occurrence, but Node are loaded from top to bottom
            #     # try to find the correct match
            #     line_number = lines_with_node_name[
            #         node_occurrence[item.launch_name]
            #     ][0]
            #     line_text = lines_with_node_name[node_occurrence[item.launch_name]][
            #         1
            #     ]

            # range in text where the node appears
            self.file_range = {
                "startLineNumber": line_number,
                "endLineNumber": line_number,
                "startColumn": start_column,
                "endColumn": end_column,
            }

    # def __del__(self):
    #     try:
    #         LaunchNodeWrapper._unique_names.remove(self.unique_name)
    #         Log.debug(f"removed from unique {self.unique_name}")
    #     except (ValueError, KeyError):
    #         # remove index
    #         LaunchNodeWrapper._remapped_names[self.node_name].remove(
    #             self.unique_name)
    #         Log.debug(f"removed from remapped {self.unique_name}")

    def _get_node_executable(self):
        if getattr(self, 'executable', ''):
            return self.executable
        result = ''
        # no name was set for Node or ExecuteProcess => use executable
        if not result:
            result = getattr(self._entity, '_Node__executable', '')
        # no name was set for Node or ExecuteProcess => use node_executable; before foxy
        if not result:
            result = getattr(self._entity, '_Node__node_executable', '')
        if result:
            if not isinstance(result, str):
                result = launch.utilities.perform_substitutions(
                    self._launch_context, result)
        self.executable = result
        return result

    def _get_launch_prefix(self) -> Union[str, None]:
        prefix = getattr(self._entity, 'prefix', None)
        if not prefix:
            prefix = getattr(self._entity, 'launch-prefix', None)
        if prefix:
            prefix = launch.utilities.perform_substitutions(
                self._launch_context, self._entity.prefix)
        return prefix

    def _get_respawn(self) -> bool:
        return getattr(self._entity, '_ExecuteProcess__respawn', False)

    def _get_respawn_delay(self) -> Union[float, None]:
        return getattr(self._entity, '_ExecuteProcess__respawn_delay', None)

    def _get_parameter_arguments(self):
        pp = getattr(self._entity, '_Node__expanded_parameter_arguments', [])
        if len(pp) == 0 and hasattr(self._entity, "parameters"):
            # returns the parameter of the composable node that is different from the rest!
            if self._entity.parameters is not None:
                return self._entity.parameters
        return pp

    def _get_arguments(self):
        return getattr(self._entity, '_Node__arguments', [])

    def _get_node_package(self) -> str:
        """Getter for node_package."""
        result = getattr(self._entity, '_Node__package', '')
        return result

    def _get_namespace(self) -> str:
        result = getattr(self._entity, 'expanded_node_namespace', None)
        if result is None:
            result = perform_to_string(self._launch_context, getattr(self._entity, 'node_namespace', None))
        if result is None or result == launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAMESPACE:
            result = ''
        base_ns = self._launch_context.launch_configurations.get('ros_namespace', None)
        result = make_namespace_absolute(prefix_namespace(base_ns, result))
        return result

    def _get_name(self) -> Tuple[str, str]:
        name_configured = None
        result = ''
        # first get name from launch.ExecuteProcess
        result = getattr(self._entity, 'name', '')
        # get name from launch_ros.actions.Node
        if not result:
            result = getattr(self._entity, 'node_name', '')
        if result:
            if not isinstance(result, str):
                result = launch.utilities.perform_substitutions(
                    self._launch_context, result)
            if result.endswith(launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAME):
                result = ''
        if result:
            name_configured = result
        if not result:
            # use executable as name
            result = self._get_node_executable()
        # try to create the name from command line
        if not result:
            result = self._get_name_from_cmd()
            if result:
                Log.info(f"Nodename '{result}' from cmd")
        # check for valid namespace
        if result and not result.startswith(SEP):
            ns = self._get_namespace()
            result = names.ns_join(ns, result)
            if name_configured:
                name_configured = result
        # if only the name is set in the launch file. 'node_name' returns name with unspecified namespace
        result = result.replace(
            f"{launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAMESPACE}/", '')
        if name_configured:
            name_configured = result
        if not result:
            Log.warn("No name for node found: %s %s" %
                     (type(self._entity), dir(self._entity)))
        # check for unique name
        unique_name = result
        # remove unique name generation... it cause problems while reload launch file
        # since we need the node to detect changes, the nodes from new launch file increase the index

        # if result in LaunchNodeWrapper._unique_names:
        #     # the name already exists! create a unique one
        #     name_set = set()
        #     if result in LaunchNodeWrapper._remapped_names:
        #         name_set = LaunchNodeWrapper._remapped_names[result]
        #     else:
        #         LaunchNodeWrapper._remapped_names[result] = name_set
        #     index = 2
        #     unique_name = f"{result}_{index}"
        #     while unique_name in name_set:
        #         index += 1
        #         unique_name = f"{result}_{index}"
        #     name_set.add(unique_name)
        # else:
        #     LaunchNodeWrapper._unique_names.add(result)
        Log.debug(f"create node wrapper with name '{result}'")
        return (result, unique_name, name_configured)

    def _get_name_from_cmd(self):
        result = ''
        cmd_list = getattr(self._entity, 'cmd', [])
        if cmd_list:
            result = launch.utilities.perform_substitutions(
                self._launch_context, cmd_list[0])
        result = os.path.basename(result.replace(' ', '_'))
        return result

    def get_composed_load_request(self):
        return self._load_node_request

    def _create_composed_load_request(self, context):
        composable_node_description: launch_ros.descriptions.ComposableNode = self._entity
        request = composition_interfaces.srv.LoadNode.Request()
        request.package_name = perform_substitutions(context, composable_node_description.package)
        request.plugin_name = perform_substitutions(context, composable_node_description.node_plugin)
        if composable_node_description.node_name is not None:
            request.node_name = perform_substitutions(context, composable_node_description.node_name)
        expanded_ns = composable_node_description.node_namespace
        if expanded_ns is not None:
            expanded_ns = perform_substitutions(context, expanded_ns)
        base_ns = context.launch_configurations.get('ros_namespace', None)
        combined_ns = make_namespace_absolute(prefix_namespace(base_ns, expanded_ns))
        if combined_ns is not None:
            request.node_namespace = combined_ns
        # request.log_level = perform_substitutions(context, node_description.log_level)
        remappings = []
        global_remaps = context.launch_configurations.get('ros_remaps', None)
        if global_remaps:
            remappings.extend([f'{src}:={dst}' for src, dst in global_remaps])
        if composable_node_description.remappings:
            remappings.extend([
                f'{perform_substitutions(context, src)}:={perform_substitutions(context, dst)}'
                for src, dst in composable_node_description.remappings
            ])
        if remappings:
            request.remap_rules = remappings
        params_container = context.launch_configurations.get('global_params', None)
        parameters = []
        if params_container is not None:
            for param in params_container:
                if isinstance(param, tuple):
                    subs = normalize_parameter_dict({param[0]: param[1]})
                    parameters.append(subs)
                else:
                    param_file_path = Path(param).resolve()
                    assert param_file_path.is_file()
                    subs = ParameterFile(param_file_path)
                    parameters.append(subs)
        if composable_node_description.parameters is not None:
            parameters.extend(list(composable_node_description.parameters))
        if parameters:
            request.parameters = [
                param.to_parameter_msg() for param in to_parameters_list(
                    context, request.node_name, expanded_ns,
                    evaluate_parameters(context, parameters)
                )
            ]
        if composable_node_description.extra_arguments is not None:
            request.extra_arguments = [
                param.to_parameter_msg() for param in to_parameters_list(
                    context, request.node_name, expanded_ns,
                    evaluate_parameters(
                        context, composable_node_description.extra_arguments
                    )
                )
            ]
        return request


class LaunchConfig(object):
    '''
    A class to handle the ROS configuration stored in launch file.
    '''

    def __init__(self, launch_file: str, *, context=None, package=None, daemonuri='', launch_arguments: List[Tuple[Text, Text]] = []):
        '''
        Creates the LaunchConfig object. The launch file will be not loaded on
        creation, first on request of roscfg value.

        :param str launch_file: The absolute or relative path with the launch file.
                                By using relative path a package must be valid for
                                remote launches.
        :param package: the package containing the launch file. If None the
                        launch_file will be used to determine the launch file.
                        No remote launches a possible without a valid package.
        :type package: str or None
        :param str daemonuri: daemon where to start the nodes of this launch file.
        :raise roslaunch.XmlParseException: if the launch file can't be found.
        '''
        self.__launch_file = launch_file
        self.__package = ros_pkg.get_name(os.path.dirname(self.__launch_file))[
            0] if package is None else package
        self.launch_type = 'python'
        if self.__launch_file.endswith('.xml') or self.__launch_file.endswith('.launch'):
            self.launch_type = 'xml'
        self._nodes: List[LaunchNodeWrapper] = []
        self.__nmduri = daemonuri
        self.provided_launch_arguments = launch_arguments
        self.launch_arguments: List[LaunchArgument] = []
        argv = sys.argv[1:]
        argv.extend(["%s:=%s" % (name, value)
                     for (name, value) in launch_arguments])
        self.__launch_context = context
        if context is None:
            self.__launch_context = LaunchContext(argv=argv)
        for (name, value) in launch_arguments:
            name_normalized = perform_substitutions(self.__launch_context,
                                                    launch.utilities.normalize_to_list_of_substitutions(name))
            self.__launch_context.launch_configurations[name_normalized] = value
            self.launch_arguments.append(LaunchArgument(name_normalized, value))
        self.__launch_description = get_launch_description_from_any_launch_file(self.filename)
        self._included_files: List[LaunchIncludedFile] = []
        self.__launch_description.launch_name = self.filename
        self.load()
        self.argv = None
        if self.argv is None:
            self.argv = []
        self.__reqTested = False
        self.__argv_values = dict()
        self.__launch_id = '%.9f' % time.time()
        self._robot_description = None
        self._capabilities = None
        self.resolve_dict = {}
        self.changed = True

    def __del__(self):
        Log.info(f"delete Launch config {self.filename}")
        self._nodes.clear()

    @property
    def context(self) -> LaunchContext:
        return self.__launch_context

    @property
    def daemonuri(self) -> str:
        '''
        :return: Returns the URI (host) of daemon where the node of this config will be started.
        :rtype: str
        '''
        return self.__nmduri

    @property
    def roscfg(self) -> launch.LaunchDescription:
        '''
        Holds a loaded launch configuration. It raises a LaunchConfigException on load error.
        :return: a previously loaded ROS configuration
        '''
        if self.__launch_description is not None:
            return self.__launch_description
        else:
            result, _ = self.load(self.argv)
            if not result:
                raise LaunchConfigException("not all argv are set properly!")
            return self.__launch_description

    def find_definition(self, content, identifier, start=0, include_close_bracket=True):
        # searches for identifier in launch file.
        # we use it to find e.g. include file directives
        identifier_pattern = None
        if self.launch_type == 'xml':
            # TODO: search for includes in XML files
            return -1, -1, ""
        elif self.launch_type == 'python':
            if identifier == 'include':
                identifier_pattern = re.compile(
                    rf"[^#]\sIncludeLaunchDescription\s*?\(", re.DOTALL | re.MULTILINE | re.S)
            elif identifier == 'group':
                identifier_pattern = re.compile(rf"[^#]\sGroupAction\s*?\(", re.DOTALL | re.MULTILINE | re.S)
            else:
                identifier_pattern = re.compile(rf"[^#]\s{identifier}\s*?\(", re.DOTALL | re.MULTILINE | re.S)
        else:
            identifier_pattern = re.compile(rf"[^#]\s{identifier}\s*?\(", re.DOTALL | re.MULTILINE | re.S)
        line_number = -1
        end_position = -1
        raw_text = ""
        match = identifier_pattern.search(content, start)
        if match is not None:
            open_brackets = 0
            line_number = content[:match.start()].count('\n') + 1
            end_position = match.end()
            raw_text = content[match.start():match.end()]
            if include_close_bracket:
                for idx in range(match.end()+1, len(content)-1):
                    if content[idx] == '(':
                        open_brackets += 1
                    if content[idx] == ')':
                        open_brackets -= 1
                        if open_brackets < 0:
                            end_position = idx
                            raw_text = content[match.start():idx]
                            break
        return line_number, end_position, raw_text

    def unload(self):
        Log.info(f"unload launch file: {self.filename}")
        self.launch_arguments.clear()
        self._included_files.clear()
        self._nodes.clear()

    def load(self) -> None:
        Log.info(
            f"load launch file: {self.filename}, arguments: {[f'{v.name}:={v.value}' for v in self.launch_arguments]}")
        environment = os.environ.copy()
        self._load(current_file=self.filename)
        # restore environment after file was loaded. To avoid interaction if multiple files are loaded.
        os.environ.clear()
        os.environ.update(environment)

    def _load(self, sub_obj: Union[None, List[launch.frontend.Entity]] = None, *, launch_description=None, current_file: str = '', indent: str = '', launch_file_obj: Union[LaunchIncludedFile, None] = None, depth: int = -1, start_position_in_file=0, timer_period=0) -> None:
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {indent}perform file {current_file}")
        current_launch_description = launch_description
        file_content = ""
        launch_prefix = ""
        environment = os.environ
        if sub_obj is None:
            sub_obj = self.__launch_description
            self.context.extend_locals({'launch_file_path': self.filename})
            self.context.extend_locals({'launch_arguments': self.launch_arguments})
            self.context.extend_locals({v.name: v.value for v in self.launch_arguments})
        if current_file:
            self.context.extend_locals({'current_launch_file_path': current_file})
            with open(current_file, 'r') as f:
                file_content = f.read()
        if current_launch_description is None:
            current_launch_description = self.__launch_description

        # import traceback
        # print(traceback.format_stack())
        # print("Launch arguments:")
        # for la in self.__launch_description.get_launch_arguments():
        #     print(la.name, launch.utilities.perform_substitutions(self.context, la.default_value))
        position_in_file = start_position_in_file
        entities = None
        if hasattr(sub_obj, 'get_sub_entities'):
            entities = getattr(sub_obj, 'get_sub_entities')()
        elif hasattr(sub_obj, 'entities'):
            entities = getattr(sub_obj, 'entities')
        else:
            entities = sub_obj
        if entities is None:
            return
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {indent}entities: {entities}")
        for entity in entities:
            if PRINT_DEBUG_LOAD:
                print(f"  ***debug launch loading: {indent}perform entity: {entity}")
            if hasattr(entity, "condition") and entity.condition:
                # check for available condition
                # if condition does not evaluate to True we have to parse for
                #   1. included file: we track the line of include directives
                #   2. GroupActions: we need line number for recursive _load() call
                if not entity.condition.evaluate(self.context):
                    if isinstance(entity, launch.actions.include_launch_description.IncludeLaunchDescription):
                        # perform search
                        inc_file_exists = False
                        file_size = -1
                        entity.execute(self.context)
                        inc_file_name = perform_to_string(
                            self.context, entity.launch_description_source.location)
                        used_path = inc_file_name
                        if os.path.exists(inc_file_name):
                            inc_file_exists = True
                            file_size = os.path.getsize(inc_file_name)
                            used_path = os.path.realpath(inc_file_name)
                        include_line_number, position_in_file, raw_text = self.find_definition(
                            file_content, 'include', position_in_file)
                        launch_inc_file = LaunchIncludedFile(path=current_file,
                                                             line_number=include_line_number,
                                                             inc_path=used_path,
                                                             exists=inc_file_exists,
                                                             raw_inc_path=raw_text,
                                                             rec_depth=depth+1,
                                                             args=[],
                                                             default_inc_args=[],
                                                             size=file_size,
                                                             conditional_excluded=True)
                        self._included_files.append(launch_inc_file)
                    elif isinstance(entity, launch.actions.group_action.GroupAction):
                        include_line_number, position_in_file, raw_text = self.find_definition(
                            file_content, 'group', position_in_file)
                    continue
            if isinstance(entity, launch_ros.actions.node.Node):
                try:
                    if PRINT_DEBUG_LOAD:
                        print(f"  ***debug launch loading: {indent}  parse node: {entity._Node__node_executable}")
                    entity._perform_substitutions(self.context)
                    if PRINT_DEBUG_LOAD:
                        print(f"  ***debug launch loading: {indent}  node after subst: {entity._Node__node_executable}")
                    # actions = entity.execute(self.context)
                    node = LaunchNodeWrapper(
                        entity, current_launch_description, self.context, environment=environment, position_in_file=position_in_file)
                    node.timer_period = timer_period
                    if launch_prefix:
                        node.launch_prefix = launch_prefix
                    self._nodes.append(node)
                    # for action in actions:
                    #    if isinstance(action, launch_ros.actions.LoadComposableNodes):

                    if isinstance(entity, launch_ros.actions.ComposableNodeContainer):
                        for cn in entity._ComposableNodeContainer__composable_node_descriptions:
                            node = LaunchNodeWrapper(cn, current_launch_description, self.context,
                                                     composable_container=node.unique_name, environment=environment, position_in_file=position_in_file)
                            node.timer_period = timer_period
                            self._nodes.append(node)
                except (SubstitutionFailure, PackageNotFoundError) as err:
                    raise err
                except:
                    import traceback
                    print(traceback.format_exc())
            elif isinstance(entity, launch_ros.actions.load_composable_nodes.LoadComposableNodes):
                if PRINT_DEBUG_LOAD:
                    print(
                        f"  ***debug launch loading: {indent}  load composable nodes: {len(entity._LoadComposableNodes__composable_node_descriptions)}")
                for cn in entity._LoadComposableNodes__composable_node_descriptions:
                    container_name = ""
                    if isinstance(entity._LoadComposableNodes__target_container, launch_ros.actions.ComposableNodeContainer):
                        node = LaunchNodeWrapper(
                            entity._LoadComposableNodes__target_container, current_launch_description, self.context, environment=environment, position_in_file=position_in_file)
                        node.timer_period = timer_period
                        self._nodes.append(node)
                        container_name = node.node_name
                    else:
                        subs = normalize_to_list_of_substitutions(entity._LoadComposableNodes__target_container)
                        container_name = make_namespace_absolute(perform_substitutions(self.context, subs))
                    node = LaunchNodeWrapper(cn, current_launch_description, self.context,
                                             composable_container=container_name, environment=environment, position_in_file=position_in_file)
                    node.timer_period = timer_period
                    self._nodes.append(node)
            elif isinstance(entity, launch.actions.execute_process.ExecuteProcess):
                if PRINT_DEBUG_LOAD:
                    print(f"  ***debug launch loading: {indent}  add execute process")
                node = LaunchNodeWrapper(entity, current_launch_description,
                                         self.context, environment=environment, position_in_file=position_in_file)
                node.timer_period = timer_period
                self._nodes.append(node)
            elif isinstance(entity, launch.actions.declare_launch_argument.DeclareLaunchArgument):
                # if entity.default_value is not None:
                #     print('  perform ARG:', entity.name, launch.utilities.perform_substitutions(
                #         self.context, entity.default_value))
                # cfg_actions = entity.execute(self.context)
                # if cfg_actions is not None:
                #     for cac in cfg_actions:
                #         print("  ***debug launch loading action: ", indent, '->', type(cac), cac)
                if launch_file_obj:
                    if PRINT_DEBUG_LOAD:
                        print(f"  ***debug launch loading: {indent} add declared argument: {entity.name}")
                    la = LaunchArgument(name=perform_to_string(self.context, entity.name),
                                        value="",
                                        default_value=perform_to_string(self.context, entity.default_value),
                                        description=perform_to_string(self.context, entity.description),
                                        choices=entity.choices)
                    launch_file_obj.default_inc_args.append(la)
            elif isinstance(entity, launch.actions.include_launch_description.IncludeLaunchDescription):
                # launch.actions.declare_launch_argument.DeclareLaunchArgument
                try:
                    cfg_actions = entity.execute(self.context)
                    if PRINT_DEBUG_LOAD:
                        print(
                            f"  ***debug launch loading: {indent} include file: {entity.launch_description_source.location}")
                    inc_file_exists = False
                    file_size = -1
                    used_path = entity.launch_description_source.location
                    if os.path.exists(entity.launch_description_source.location):
                        inc_file_exists = True
                        used_path = os.path.realpath(used_path)
                        file_size = os.path.getsize(used_path)
                    include_line_number, position_in_file, raw_text = self.find_definition(
                        file_content, 'include', position_in_file)
                    inc_launch_arguments = []
                    if cfg_actions is not None:
                        for cac in cfg_actions:
                            if isinstance(cac, launch.actions.set_launch_configuration.SetLaunchConfiguration):
                                cac.execute(self.context)
                                arg_name = cac.name
                                if isinstance(cac.name, List):
                                    arg_name = cac.name[0].perform(self.context)
                                arg_value = cac.value
                                if isinstance(cac.value, List):
                                    arg_value = cac.value[0].perform(self.context)
                                if PRINT_DEBUG_LOAD:
                                    print(
                                        f"  ***debug launch loading: {indent}  add launch config: {arg_name}: {arg_value}")
                                inc_launch_arguments.append(LaunchArgument(name=arg_name, value=arg_value))
                    inc_launch_arguments_def = []
                    launch_inc_file = LaunchIncludedFile(path=current_file,
                                                         line_number=include_line_number,
                                                         inc_path=used_path,
                                                         exists=inc_file_exists,
                                                         raw_inc_path=raw_text,
                                                         rec_depth=depth+1,
                                                         args=inc_launch_arguments,
                                                         default_inc_args=inc_launch_arguments_def,
                                                         size=file_size
                                                         )
                    self._included_files.append(launch_inc_file)
                    self._load(entity, launch_description=entity, current_file=entity._get_launch_file(),
                               indent=indent+'  ', launch_file_obj=launch_inc_file, depth=depth+1, start_position_in_file=0, timer_period=timer_period)
                    if current_file:
                        self.context.extend_locals({'current_launch_file_path': current_file})
                except launch.invalid_launch_file_error.InvalidLaunchFileError as err:
                    raise Exception('%s (%s)' % (
                        err, entity.launch_description_source.location))
            elif isinstance(entity, launch.actions.group_action.GroupAction):
                if current_file:
                    self.context.extend_locals({'current_launch_file_path': current_file})
                include_line_number, position_in_file, raw_text = self.find_definition(
                    file_content, 'group', position_in_file, include_close_bracket=False)
                self._load(entity, launch_description=current_launch_description,
                           current_file=current_file, indent=indent+'  ', launch_file_obj=launch_file_obj, depth=depth, start_position_in_file=position_in_file, timer_period=timer_period)
            elif isinstance(entity, launch.actions.timer_action.TimerAction):
                if PRINT_DEBUG_LOAD:
                    print(f"  ***debug launch loading: {indent} timer period: {entity.period}")
                period = entity.period
                if not isinstance(period, (float, int)):
                    period = float(perform_substitutions(self.context, entity.period))
                if PRINT_DEBUG_LOAD:
                    print(f"  ***debug launch loading: {indent} timer period (resolved): {period}")
                if PRINT_DEBUG_LOAD:
                    print(f"  ***debug launch loading: {indent} actions count: {len(entity.actions)}")
                self._load(entity.actions, launch_description=current_launch_description, current_file=current_file,
                           indent=indent+'  ', launch_file_obj=launch_file_obj, depth=depth, start_position_in_file=position_in_file, timer_period=period)
                # period: Union[float, SomeSubstitutionsType],
                # actions: Iterable[LaunchDescriptionEntity],
                # cancel_on_shutdown: Union[bool, SomeSubstitutionsType] = True,
            elif isinstance(entity, launch.actions.set_environment_variable.SetEnvironmentVariable):
                if hasattr(entity, 'execute'):
                    entity.execute(self.context)
                name = perform_substitutions(self.context, getattr(entity, 'name', ''))
                value = perform_substitutions(self.context, getattr(entity, 'value', ''))
                environment[name] = value
            elif isinstance(entity, launch.actions.unset_environment_variable.UnsetEnvironmentVariable):
                if hasattr(entity, 'execute'):
                    entity.execute(self.context)
                name = perform_substitutions(self.context, getattr(entity, 'name', ''))
                if name in environment:
                    del environment[name]
            elif hasattr(entity, 'execute'):
                if PRINT_DEBUG_LOAD:
                    print(f"  ***debug launch loading: {indent} parse execute entity: {entity}; {dir(entity)}")
                try:

                    # entity._perform_substitutions(self.context)
                    exec_result = entity.execute(self.context)
                    if not exec_result:
                        name = perform_substitutions(self.context, getattr(entity, 'name', ''))
                        if name == "launch-prefix":
                            launch_prefix = perform_substitutions(self.context, getattr(entity, 'value', ''))
                    else:
                        if PRINT_DEBUG_LOAD:
                            print(f"  ***debug execute result: {exec_result}; {dir(exec_result)}")
                        if not isinstance(exec_result, List):
                            exec_result = [exec_result]
                        self._load(exec_result, launch_description=current_launch_description,
                                   current_file=current_file, indent=indent+'  ', launch_file_obj=launch_file_obj, depth=depth, start_position_in_file=position_in_file, timer_period=timer_period)

                except:
                    import traceback
                    print(traceback.format_exc())
            else:
                print(f"  ***debug launch loading: {indent} unknown entity: {entity}")
                self._load(entity, launch_description=current_launch_description,
                           current_file=current_file, indent=indent+'  ', launch_file_obj=launch_file_obj, depth=depth+1, start_position_in_file=position_in_file, timer_period=timer_period)
                if current_file:
                    self.context.extend_locals({'current_launch_file_path': current_file})

    def nodes(self) -> List[LaunchNodeWrapper]:
        return self._nodes

    @property
    def filename(self) -> Text:
        '''
        Returns an existing path with file name or an empty string.
        '''
        if os.path.isfile(self.__launch_file):
            return self.__launch_file
        elif self.package_name:
            try:
                return roslib.packages.find_resource(self.package_name, self.launch_name).pop()
            except Exception:
                raise LaunchConfigException(f'launch file {self.launch_name} not found!')
        raise LaunchConfigException(f'launch file {self.__launch_file} not found!')

    @property
    def launch_name(self) -> Text:
        '''
        Returns the name of the launch file with extension, e.g. 'test.launch'
        '''
        return os.path.basename(self.__launch_file)

    @property
    def package_name(self) -> Union[Text, None]:
        '''
        Returns the name of the package containing the launch file or None.
        '''
        return self.__package

    @classmethod
    def get_launch_arguments(cls, context: LaunchContext, filename: str, *, provided_args: Union[List, None]) -> List[LaunchArgument]:
        '''
        :return: a list with args being used in the roslaunch file.
        '''
        declared_launch_arguments: List[
            Tuple[launch.actions.DeclareLaunchArgument, List[launch.actions.IncludeLaunchDescription]]] = []
        # based on LaunchDescription.get_launch_arguments()

        def process_entities(entities, *, _conditional_inclusion=False, nested_ild_actions=None):
            next_nested_ild_actions = nested_ild_actions
            for entity in entities:
                if isinstance(entity, launch.actions.DeclareLaunchArgument):
                    # Avoid duplicate entries with the same name.
                    if entity.name in (e.name for e, _ in declared_launch_arguments):
                        continue
                    # Stuff this contextual information into the class for
                    # potential use in command-line descriptions or errors.
                    entity._conditionally_included = _conditional_inclusion
                    entity._conditionally_included |= entity.condition is not None
                    declared_launch_arguments.append((entity, nested_ild_actions))
                if isinstance(entity, launch.actions.ResetLaunchConfigurations):
                    # Launch arguments after this cannot be set directly by top level arguments
                    return declared_launch_arguments
                elif next_nested_ild_actions is not None:
                    next_nested_ild_actions = nested_ild_actions
                    if isinstance(entity, launch.actions.IncludeLaunchDescription):
                        next_nested_ild_actions.append(entity)
                    process_entities(
                        entity.describe_sub_entities(),
                        _conditional_inclusion=False,
                        nested_ild_actions=next_nested_ild_actions)
                    for conditional_sub_entity in entity.describe_conditional_sub_entities():
                        process_entities(
                            conditional_sub_entity[1],
                            _conditional_inclusion=True,
                            nested_ild_actions=next_nested_ild_actions)
            return declared_launch_arguments

        launch_description = get_launch_description_from_any_launch_file(filename)

        launch_arguments: List[launch.actions.DeclareLaunchArgument] = []
        if provided_args is None:
            launch_arguments = [item[0] for item in process_entities(launch_description.entities)]
        else:
            # use recursive method of ROS to get all default values
            launch_arguments = launch_description.get_launch_arguments()
        result = []
        for argument_action in launch_arguments:
            value = None
            if provided_args is not None:
                for provided_arg in provided_args:
                    if argument_action.name == provided_arg.name and hasattr(provided_arg, "value"):
                        value = provided_arg.value
                        break

            default_value = None
            if argument_action.default_value is not None:
                default_value = launch.utilities.perform_substitutions(context, argument_action.default_value)
            arg = LaunchArgument(name=argument_action.name,
                                    value=value if value is not None else default_value,
                                    default_value=default_value,
                                    description=argument_action.description,
                                    choices=argument_action.choices)
            result.append(arg)
        return result

    def get_node(self, name: str) -> Union[LaunchNodeWrapper, None]:
        '''
        Returns a configuration node for a given node name.
        '''
        for item in self.nodes():
            if (item.unique_name == name):
                return item
        Log.warn("Node '%s' NOT found" % name)
        return None

    def run_node(self, name: str, ignore_timer=False) -> str:
        '''
        Start a node local

        :return: path of executable or empty string on load composable node
        :raise exceptions.StartException: on errors
        :raise exceptions.BinarySelectionRequest: on multiple binaries
        '''
        node: LaunchNodeWrapper = self.get_node(name)
        if node is None:
            raise exceptions.StartException(f"Node '{name}' in '{self.filename}' not found!")
        if node.timer_period > 0 and not ignore_timer:
            t = threading.Timer(node.timer_period, self.run_node, args=(name, True))
            t.start()
            # TODO: add executable to observed files
            return f"{name} will be started in {node.timer_period} seconds"
        if node.composable_container:
            # load plugin in container
            Log.info(f"Load node='{node.unique_name}'; as plugin into container='{node.composable_container}';")
            # skip check if container is running, it is done by the GUI
            self.run_composed_node(node)
            return ''

        # run on local host
        # run get_cmd() before create new_env since get_cmd() extends os.environ
        screen_prefix = screen.get_cmd(node.unique_name)
        # set environment
        new_env = dict(os.environ) if node.env is None else dict(node.env)
        # set display variable to local display
        if 'DISPLAY' in new_env:
            if not new_env['DISPLAY'] or new_env['DISPLAY'] == 'remote':
                del new_env['DISPLAY']
        else:
            new_env['DISPLAY'] = ':0'
        # add environment from launch
        if node.additional_env:
            new_env.update(dict(node.additional_env))
        if node.node_namespace:
            new_env['ROS_NAMESPACE'] = node.node_namespace
        # set logging
        if node.output_format:
            new_env['ROSCONSOLE_FORMAT'] = '%s' % node.output_format
        # if node.loglevel:
        #     new_env['ROSCONSOLE_CONFIG_FILE'] = _rosconsole_cfg_file(
        #         node.package, node.loglevel)
        # handle respawn
        respawn_prefix = ''
        if node.respawn:
            if node.respawn_delay and node.respawn_delay > 0:
                new_env['RESPAWN_DELAY'] = '%d' % node.respawn_delay
            # TODO
            # respawn_params = _get_respawn_params(node.fullname, node.params)
            # if respawn_params['max'] > 0:
            #     new_env['RESPAWN_MAX'] = '%d' % respawn_params['max']
            # if respawn_params['min_runtime'] > 0:
            #     new_env['RESPAWN_MIN_RUNTIME'] = '%d' % respawn_params['min_runtime']
            respawn_prefix = f"{RESPAWN_SCRIPT}"

        launch_prefix = ''
        if node.launch_prefix:
            launch_prefix = node.launch_prefix
        # TODO: check for HOSTNAME
        # start
        executable_path = ''
        if node.cmd:
            executable_path = node.cmd.split()[0]
        Log.info(f"{screen_prefix} {respawn_prefix} {launch_prefix} {node.cmd} (launch_file: '{node.launch_name}')")
        Log.debug(f"environment while run node '{node.unique_name}': '{new_env}'")
        SupervisedPopen(' '.join([screen_prefix, respawn_prefix, launch_prefix, node.cmd]), cwd=node.cwd, shell=True, env=new_env,
                        object_id=f"run_node_{node.unique_name}", description=f"Run [{node.package_name}]{node.executable}")
        return executable_path

    def run_composed_node(self, node: LaunchNodeWrapper):
        # Create a client to load nodes in the target container.
        client_load_node = nmd.ros_node.create_client(
            composition_interfaces.srv.LoadNode, f'{node.composable_container}/_container/load_node')
        request = node.get_composed_load_request()
        service_load_node_name = f'{node.composable_container}/_container/load_node'
        Log.debug(f" -> load composed node to '{service_load_node_name}'")
        response = nmd.launcher.call_service(
            service_load_node_name, composition_interfaces.srv.LoadNode, request)
        if response is None:
            error_msg = f"Failed to load service '{request.node_name}' of type '{request.plugin_name}' in container '{node.composable_container}': None as service response"
            Log.error(error_msg)
            raise exceptions.StartException(error_msg)
        Log.debug(f"  <- load composed node: response received: {response} {dir(response)}")
        node_name = response.full_node_name if response.full_node_name else request.node_name
        nmd.ros_node.destroy_client(client_load_node)
        if response.success:
            # if node_name is not None:
            #     add_node_name(context, node_name)
            #     node_name_count = get_node_name_count(context, node_name)
            #     if node_name_count > 1:
            #         container_logger = launch.logging.get_logger(self.__target_container.name)
            #         container_logger.warning(
            #             'there are now at least {} nodes with the name {} created within this '
            #             'launch context'.format(node_name_count, node_name)
            #         )
            Log.info(f"Loaded node '{response.full_node_name}' in container '{node.composable_container}'")
        else:
            error_msg = f"Failed to load node '{node_name}' of type '{request.plugin_name}' in container '{node.composable_container}': {response.error_message}"
            Log.error(error_msg)
            raise exceptions.StartException(error_msg)
