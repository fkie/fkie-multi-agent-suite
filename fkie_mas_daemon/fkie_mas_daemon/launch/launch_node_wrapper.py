# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import re
from pathlib import Path

import composition_interfaces.srv
import launch
import launch.utilities
import launch_ros
import ruamel.yaml
from fkie_mas_pylib.defines import SEP
from fkie_mas_pylib.interface.launch_interface import LaunchArgument, LaunchNodeInfo
from fkie_mas_pylib.interface.runtime_interface import RosParameter
from fkie_mas_pylib.logging.logging import Log
from launch.utilities import perform_substitutions
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.utilities import make_namespace_absolute, prefix_namespace, to_parameters_list
from launch_ros.utilities.evaluate_parameters import evaluate_parameters
from launch_ros.utilities.normalize_parameters import normalize_parameter_dict

from fkie_mas_pylib import names

from .launch_load_state import LaunchLoadState
from .utils import perform_to_string, perform_to_tuple_list

PRINT_DEBUG_LOAD = False


class LaunchNodeWrapper(LaunchNodeInfo):
    # _unique_names: Set[str] = set()
    # _remapped_names: Dict[str, Set[str]] = {}

    def __init__(
        self,
        entity: launch.actions.ExecuteProcess,
        load_state: LaunchLoadState,
        *,
        launch_context: launch.LaunchContext,
        #                 launch_description: Union[launch.LaunchDescription, launch.actions.IncludeLaunchDescription],
        composable_container: str = None,
        #                environment: Dict = {},
        #                remove_environment: List[str] = None
    ) -> None:
        self._entity = entity
        self._launch_description = load_state.launch_description
        self._launch_context = launch_context
        self.timer_period = 0
        if isinstance(self._entity, launch_ros.actions.Node):
            # Prepare the ros_specific_arguments list and add it to the context so that the
            # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
            ros_specific_arguments: dict[str, str | list[str]] = {}
            if self._entity._Node__node_name is not None:
                ros_specific_arguments["name"] = f"__node:={self._entity._Node__expanded_node_name}"
            if self._entity._Node__expanded_node_namespace != "":
                ros_specific_arguments["ns"] = f"__ns:={self._entity._Node__expanded_node_namespace}"

            # Give extensions a chance to prepare for execution
            for extension in self._entity._Node__extensions.values():
                cmd_extension, ros_specific_arguments = extension.prepare_for_execute(
                    self._launch_context, ros_specific_arguments, self
                )
                self._entity._Node.cmd.extend(cmd_extension)

            self._launch_context.extend_locals({"ros_specific_arguments": ros_specific_arguments})

        node_name, unique_name, name_configured = self._get_name()
        LaunchNodeInfo.__init__(self, unique_name=unique_name, node_name=node_name, name_configured=name_configured)
        if PRINT_DEBUG_LOAD:
            print("  ***debug LaunchNodeWrapper: name_configured", name_configured)
        self.node_namespace = self._get_namespace()
        self.package_name = self._get_node_package()
        self.executable = self._get_node_executable()
        self.respawn = self._get_respawn()
        self.respawn_delay = self._get_respawn_delay()
        if isinstance(self._launch_description, launch.actions.IncludeLaunchDescription):
            # we use only real path paths!
            self.file_name = self._launch_description._get_launch_file()
            self.file_name_realpath = os.path.realpath(self.file_name)
            self.launch_name = getattr(self._launch_context.locals, "launch_file_path", None)
            # add launch arguments used to load the included file
            if self._launch_description.launch_arguments:
                self.launch_context_arg = []
            for arg_name, arg_value in self._launch_description.launch_arguments:
                self.launch_context_arg.append(
                    LaunchArgument(
                        perform_to_string(self._launch_context, arg_name),
                        perform_to_string(self._launch_context, arg_value),
                    )
                )
        else:
            self.launch_name = getattr(self._launch_description, "launch_name", "")
            self.launch_context_arg = getattr(self._launch_context.locals, "launch_arguments", None)
            self.file_name = self.launch_name
        self.composable_container: str = composable_container
        self._parameters_tmp = self._get_parameter_arguments()
        self.param_file_content = {}  # used to detect changes in referenced yaml files
        self.parameters = []
        for p in self._parameters_tmp:
            if isinstance(p, tuple) and p[0].startswith("/"):
                if os.path.exists(p[0]):
                    with open(p[0]) as tmp_param_file:
                        try:
                            yaml = ruamel.yaml.YAML(typ="base")
                            self.parameters.append(RosParameter(node_name, p[0], yaml.load(tmp_param_file)))
                            tmp_param_file.seek(0)
                            self.param_file_content[p[0]] = tmp_param_file.read()
                            continue
                        except ruamel.yaml.YAMLError:
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
                        try:
                            p_val = perform_to_string(self._launch_context, val.value, verbose=False)
                        except:
                            p_val = val.value
                    if p_name is not None and p_val is not None:
                        self.parameters.append(RosParameter(node_name, p_name, p_val))
                continue
            elif isinstance(p, ParameterFile):
                try:
                    p_file = perform_to_string(self._launch_context, p.param_file)
                    content = ""
                    try:
                        with open(p_file) as tmp_param_file:
                            try:
                                yaml = ruamel.yaml.YAML(typ="base")
                                self.parameters.append(RosParameter(node_name, p_file, yaml.load(tmp_param_file)))
                                tmp_param_file.seek(0)
                                self.param_file_content[p_file] = tmp_param_file.read()
                                continue
                            except ruamel.yaml.YAMLError as exc:
                                content = f"{exc}"
                                pass
                    except Exception as err:
                        content = f"{err}"
                    self.parameters.append(RosParameter(node_name, p_file, content))
                except:
                    pass
                continue
            print(f"ignored new parameter type: {type(p)}: {p}")
            # self.parameters.append(p)

        self.args = [perform_to_string(self._launch_context, arg) for arg in self._get_arguments() or []]
        self.cmd = perform_to_string(self._launch_context, getattr(self._entity, "cmd", None))
        if self.cmd:
            # store the content of file to detect changes on reload
            for param_file in re.findall(r"--params-file\s+([^\s]+)", self.cmd):
                if os.path.exists(param_file) and param_file not in self.param_file_content:
                    try:
                        with open(param_file) as tmp_param_file:
                            try:
                                yaml = ruamel.yaml.YAML(typ="base")
                                self.parameters.append(RosParameter(node_name, param_file, yaml.load(tmp_param_file)))
                                tmp_param_file.seek(0)
                                self.param_file_content[param_file] = tmp_param_file.read()
                                continue
                            except ruamel.yaml.YAMLError as exc:
                                content = f"{exc}"
                                pass
                    except Exception as err:
                        content = f"{err}"
                    self.parameters.append(RosParameter(node_name, param_file, content))
        self.cwd = perform_to_string(self._launch_context, getattr(self._entity, "cwd", None))
        self.remove_environment = [] if load_state.remove_environment is None else load_state.remove_environment
        # perform_to_tuple_list(self._launch_context, getattr(self._entity, 'additional_env', {}))
        self.additional_env = dict(load_state.environment)
        add_env = perform_to_tuple_list(self._launch_context, getattr(self._entity, "env", {}))
        add_env_extra = perform_to_tuple_list(self._launch_context, getattr(self._entity, "additional_env", {}))
        if add_env_extra:
            if add_env is None:
                add_env = add_env_extra
            else:
                add_env.extend(add_env_extra)
        if add_env:
            self.additional_env.update(add_env)
        self.launch_prefix = perform_to_string(self._launch_context, self._get_launch_prefix())
        self._load_node_request = None
        if self.composable_container:
            self._load_node_request = self._create_composed_load_request(self._launch_context)
        if isinstance(entity, launch_ros.actions.ComposableNodeContainer):
            self.composable_container = self.node_name

        #  remap_args: List[Tuple[str, str]] = None,
        #  output: str = '',
        #  output_format: str = '',
        #  sigterm_timeout: str = '',
        #  sigkill_timeout: str = '',
        #  on_exit: List[Any] = [],
        #  required: bool = False,
        #  file_name: str = '',
        self.file_range: dict[str, int] = {"startLineNumber": 0, "endLineNumber": 0, "startColumn": 0, "endColumn": 0}

        #  launch_context_arg: str = '',
        #  launch_name: str = ''
        #  composable_container: str = ''
        #  Search the line number of a given node in launch file

        # range in text where the node appears
        file_range = load_state.cursor().file_range(self.node_name)
        if file_range is not None:
            self.file_range = file_range
        elif PRINT_DEBUG_LOAD:
            Log.debug(f"LaunchNodeWrapper: no file range for '{self.node_name}' in {load_state.current_file}")
        self.timer_period = load_state.timer_period
        if load_state.launch_prefix:
            self.launch_prefix = load_state.launch_prefix

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
        if getattr(self, "executable", ""):
            return self.executable
        result = ""
        # no name was set for Node or ExecuteProcess => use executable
        if not result:
            result = getattr(self._entity, "_Node__executable", "")
        # no name was set for Node or ExecuteProcess => use node_executable; before foxy
        if not result:
            result = getattr(self._entity, "_Node__node_executable", "")
        if result:
            if not isinstance(result, str):
                result = launch.utilities.perform_substitutions(self._launch_context, result)
        self.executable = result
        return result

    def _get_launch_prefix(self) -> str | None:
        prefix = getattr(self._entity, "prefix", None)
        if not prefix:
            prefix = getattr(self._entity, "launch-prefix", None)
        if prefix:
            prefix = launch.utilities.perform_substitutions(self._launch_context, self._entity.prefix)
        return prefix

    def _get_respawn(self) -> bool:
        return getattr(self._entity, "_ExecuteProcess__respawn", False)

    def _get_respawn_delay(self) -> float | None:
        return getattr(self._entity, "_ExecuteProcess__respawn_delay", None)

    def _get_parameter_arguments(self):
        pp = getattr(self._entity, "_Node__expanded_parameter_arguments", [])
        if len(pp) == 0 and hasattr(self._entity, "parameters"):
            # returns the parameter of the composable node that is different from the rest!
            if self._entity.parameters is not None:
                return self._entity.parameters
        return pp

    def _get_arguments(self):
        return getattr(self._entity, "_Node__arguments", [])

    def _get_node_package(self) -> str:
        """Getter for node_package."""
        result = getattr(self._entity, "_Node__package", "")
        return perform_to_string(self._launch_context, result)

    def _get_namespace(self) -> str:
        result = getattr(self._entity, "expanded_node_namespace", None)
        if result is None and hasattr(self._entity, "node_namespace"):
            result = perform_to_string(self._launch_context, self._entity.node_namespace)
        if result is None or result == launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAMESPACE:
            result = ""
        base_ns = self._launch_context.launch_configurations.get("ros_namespace", None)
        result = make_namespace_absolute(prefix_namespace(base_ns, result))
        return result

    def _get_name(self) -> tuple[str, str]:
        name_configured = None
        result = ""
        # first get name from launch.ExecuteProcess
        result = getattr(self._entity, "name", "")
        # get name from launch_ros.actions.Node
        if not result:
            result = getattr(self._entity, "node_name", "")
        if result:
            if not isinstance(result, str):
                result = launch.utilities.perform_substitutions(self._launch_context, result)
            if result.endswith(launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAME):
                result = ""
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
        result = result.replace(f"{launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAMESPACE}/", "")
        if name_configured:
            name_configured = result
        if not result:
            Log.warn("No name for node found: %s %s" % (type(self._entity), dir(self._entity)))
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
        result = ""
        cmd_list = getattr(self._entity, "cmd", [])
        if cmd_list:
            result = launch.utilities.perform_substitutions(self._launch_context, cmd_list[0])
        result = os.path.basename(result.replace(" ", "_"))
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
        base_ns = context.launch_configurations.get("ros_namespace", None)
        combined_ns = make_namespace_absolute(prefix_namespace(base_ns, expanded_ns))
        if combined_ns is not None:
            request.node_namespace = combined_ns
        # request.log_level = perform_substitutions(context, node_description.log_level)
        remappings = []
        global_remaps = context.launch_configurations.get("ros_remaps", None)
        if global_remaps:
            remappings.extend([f"{src}:={dst}" for src, dst in global_remaps])
        if composable_node_description.remappings:
            remappings.extend(
                [
                    f"{perform_substitutions(context, src)}:={perform_substitutions(context, dst)}"
                    for src, dst in composable_node_description.remappings
                ]
            )
        if remappings:
            request.remap_rules = remappings
        params_container = context.launch_configurations.get("global_params", None)
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
                param.to_parameter_msg()
                for param in to_parameters_list(
                    context, request.node_name, request.node_namespace, evaluate_parameters(context, parameters)
                )
            ]
        if composable_node_description.extra_arguments is not None:
            request.extra_arguments = [
                param.to_parameter_msg()
                for param in to_parameters_list(
                    context,
                    request.node_name,
                    request.node_namespace,
                    evaluate_parameters(context, composable_node_description.extra_arguments),
                )
            ]
        return request
