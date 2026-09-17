# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import re
import sys
import threading
import time
from collections import defaultdict
from collections.abc import Callable
from functools import lru_cache

import composition_interfaces.srv
import launch
import launch.utilities
import launch_ros
from ament_index_python.packages import PackageNotFoundError
from fkie_mas_pylib.defines import RESPAWN_SCRIPT
from fkie_mas_pylib.interface.launch_interface import LaunchArgument, LaunchIncludedFile
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import exceptions, screen
from fkie_mas_pylib.system.supervised_popen import SupervisedPopen
from launch.launch_context import LaunchContext
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.substitutions.substitution_failure import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.utilities import make_namespace_absolute

import fkie_mas_daemon as nmd
from fkie_mas_pylib import ros_pkg

from .caches import FILE_CONTENT_CACHE, normalize_path
from .launch_argument_cache import LAUNCH_ARGUMENT_CACHE, LaunchArgTemplate
from .launch_include_index import GROUP_KIND, INCLUDE_KIND
from .launch_load_state import LaunchLoadState
from .launch_node_wrapper import LaunchNodeWrapper
from .utils import LaunchConfigException, perform_to_string

PRINT_DEBUG_LOAD = False

_component_load_timeout = float(os.environ.get("MAS_COMPONENT_LOAD_TIMEOUT", "30"))
COMPONENT_LOAD_TIMEOUT = _component_load_timeout if _component_load_timeout > 0 else None

# statically known patterns: compiled once at import time
XML_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)

# flags used for all dynamically built python launch file patterns
PYTHON_DEFINITION_FLAGS = re.DOTALL | re.MULTILINE | re.S

# The cache key of strip_xml_comments() is the complete file content, therefore
# every entry keeps two copies of a launch file alive. Keep the limit small, the
# file content itself is already cached by FILE_CONTENT_CACHE.
STRIP_COMMENTS_CACHE_SIZE = 16


@lru_cache(maxsize=STRIP_COMMENTS_CACHE_SIZE)
def strip_xml_comments(content: str) -> str:
    """
    Remove XML comments but keep the line count *and* the character offsets, so
    that both reported line numbers and positions stay valid for the raw
    content. Every comment character is replaced by a space, every line break is
    preserved. Cached, because the same file content is scanned once per include
    directive.
    """

    def replacer(match: "re.Match") -> str:
        # keep length and line breaks: offsets into the raw content stay valid
        return "".join("\n" if character == "\n" else " " for character in match.group(0))

    return XML_COMMENT_RE.sub(replacer, content)


class LaunchConfig:
    """
    A class to handle the ROS configuration stored in launch file.
    """

    # os.environ is process global: serialize launch file parsing and every
    # environment snapshot across all instances. This is the lowest level in
    # the lock hierarchy, never acquire a servicer lock while holding it.
    _LOAD_LOCK = threading.RLock()

    # dispatch table: launch file suffix -> launch type.
    # The longest matching suffix wins, so '.launch.xml' is preferred over '.xml'.
    _LAUNCH_TYPES: dict[str, str] = {
        ".launch.py": "python",
        ".launch.xml": "xml",
        ".launch.yaml": "yaml",
        ".launch.yml": "yaml",
        ".py": "python",
        ".xml": "xml",
        ".launch": "xml",
        ".yaml": "yaml",
        ".yml": "yaml",
    }

    # suffixes of _LAUNCH_TYPES, longest first. Precomputed, so that
    # resolve_launch_type() does not sort on every call. Rebuilt by
    # register_launch_type().
    _LAUNCH_TYPES_SORTED: tuple[str, ...] = ()

    # dispatch table for _load(): the order matters, the first matching type
    # wins. launch_ros Node derives from ExecuteProcess, therefore Node has to
    # be checked first.
    _ENTITY_HANDLERS: tuple[tuple[type, str], ...] = (
        (launch_ros.actions.node.Node, "_handle_node"),
        (launch_ros.actions.load_composable_nodes.LoadComposableNodes, "_handle_load_composable_nodes"),
        (launch.actions.execute_process.ExecuteProcess, "_handle_execute_process"),
        (launch.actions.declare_launch_argument.DeclareLaunchArgument, "_handle_declare_launch_argument"),
        (launch.actions.include_launch_description.IncludeLaunchDescription, "_handle_include_launch_description"),
        (launch.actions.group_action.GroupAction, "_handle_group_action"),
        (launch.actions.timer_action.TimerAction, "_handle_timer_action"),
        (launch.actions.set_environment_variable.SetEnvironmentVariable, "_handle_set_environment_variable"),
        (launch.actions.unset_environment_variable.UnsetEnvironmentVariable, "_handle_unset_environment_variable"),
        (launch.launch_description.LaunchDescription, "_handle_launch_description"),
        (launch.actions.set_launch_configuration.SetLaunchConfiguration, "_handle_set_launch_configuration"),
        (launch.actions.pop_launch_configurations.PopLaunchConfigurations, "_handle_pop_launch_configurations"),
    )

    # resolved handler per concrete entity type, avoids the isinstance chain on
    # every entity. Only written while _LOAD_LOCK is held (all loads run under
    # that lock), so no extra lock is required.
    _HANDLER_NAME_CACHE: dict[type, str] = {}

    def __init__(
        self,
        launch_file: str,
        *,
        context=None,
        package=None,
        daemonuri="",
        launch_arguments: list[tuple[str, str]] = None,
    ):
        """
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
        """
        launch_arguments = list(launch_arguments or [])

        self.__launch_file = normalize_path(launch_file)

        self.__package = ros_pkg.get_name(os.path.dirname(self.__launch_file))[0] if package is None else package

        self.load_exceptions = []
        self.launch_type = self.resolve_launch_type(self.__launch_file)
        self._nodes: list[LaunchNodeWrapper] = []
        self.__nmduri = daemonuri
        self.provided_launch_arguments = launch_arguments
        self.launch_arguments: list[LaunchArgument] = []

        # timers created by run_node() for delayed (TimerAction) nodes. They keep
        # a reference to self, therefore they have to be cancelled on unload().
        self._run_timers: list[threading.Timer] = []
        self._run_timers_lock = threading.Lock()

        argv = list(sys.argv[1:])
        argv.extend(f"{name}:={value}" for name, value in launch_arguments)

        self.__launch_context = context

        if context is None:
            self.__launch_context = LaunchContext(argv=argv)

        for name, value in launch_arguments:
            name_normalized = perform_substitutions(self.__launch_context, normalize_to_list_of_substitutions(name))

            self.__launch_context.launch_configurations[name_normalized] = value
            self.launch_arguments.append(LaunchArgument(name_normalized, value))

        self.__launch_description = get_launch_description_from_any_launch_file(self.filename)

        self._included_files: list[LaunchIncludedFile] = []
        self.__launch_description.launch_name = self.filename
        self.environment = {}

        self.load()

        self.argv = None
        if self.argv is None:
            self.argv = []

        self.__reqTested = False
        self.__argv_values = {}
        self.__launch_id = "%.9f" % time.time()
        self._robot_description = None
        self._capabilities = None
        self.resolve_dict = {}
        self.changed = True

    def __del__(self):
        # never leave a pending timer behind, it would start a node of an
        # already destroyed configuration
        self._cancel_run_timers()
        self._nodes.clear()

    # -- launch type dispatch ------------------------------------------------

    @classmethod
    def register_launch_type(cls, suffix: str, launch_type: str) -> None:
        """Register an additional launch file extension (extension point)."""
        cls._LAUNCH_TYPES[suffix.lower()] = launch_type
        # keep the precomputed order in sync
        cls._LAUNCH_TYPES_SORTED = tuple(sorted(cls._LAUNCH_TYPES, key=len, reverse=True))

    @classmethod
    def resolve_launch_type(cls, filename: str) -> str:
        """
        Determine the launch type of a launch file by its extension.
        Unknown extensions fall back to 'python', like before.
        """
        if not cls._LAUNCH_TYPES_SORTED:
            # lazy initialization on first use, avoids a module level statement
            cls._LAUNCH_TYPES_SORTED = tuple(sorted(cls._LAUNCH_TYPES, key=len, reverse=True))
        name = os.path.basename(filename).lower()
        # longest suffix wins, so '.launch.xml' is preferred over '.xml'
        for suffix in cls._LAUNCH_TYPES_SORTED:
            if name.endswith(suffix):
                return cls._LAUNCH_TYPES[suffix]
        Log.debug(f"{cls.__name__}: unknown launch file extension: {filename}, assume type 'python'")
        return "python"

    @property
    def context(self) -> LaunchContext:
        return self.__launch_context

    @property
    def daemonuri(self) -> str:
        """
        :return: Returns the URI (host) of daemon where the node of this config will be started.
        :rtype: str
        """
        return self.__nmduri

    @property
    def roscfg(self) -> launch.LaunchDescription:
        """
        Holds a loaded launch configuration. It raises a LaunchConfigException on load error.
        :return: a previously loaded ROS configuration
        """
        if self.__launch_description is not None:
            return self.__launch_description
        else:
            result, _ = self.load(self.argv)
            if not result:
                raise LaunchConfigException("not all argv are set properly!")
            return self.__launch_description

    # -- file content --------------------------------------------------------

    def _read_launch_file(self, path: str) -> str:
        """
        Content of a launch file, cached and validated with a stat fingerprint.
        The watchdog additionally invalidates the entry on change.

        An empty result also happens for files exceeding the cache size limit or
        for unreadable files. In that case no include directive can be located,
        therefore report it as a warning instead of silently returning line
        number -1 for every include.
        """
        content = FILE_CONTENT_CACHE.get_content(path, default="")
        if not content:
            Log.warn(
                f"{self.__class__.__name__}: launch file empty, too large or "
                f"unreadable, include positions are not available: {path}"
            )
        return content

    # -- definition search ---------------------------------------------------

    def _next_include(self, state: LaunchLoadState) -> tuple[int, str]:
        """
        Locate the next include directive of the current file.
        Returns line number and raw path, (-1, '') if the file has no further
        include, e.g. for includes created by an OpaqueFunction.
        """
        definition = state.includes().next(INCLUDE_KIND)
        if definition is None:
            return -1, ""
        return definition.line_number, definition.raw_path

    # -- load ----------------------------------------------------------------

    def _create_state_for_file(
        self,
        current_file: str,
        *,
        launch_description,
        indent: str,
        depth: int,
        launch_file_obj: LaunchIncludedFile | None,
        timer_period: float,
        env: dict[str, str] | None = None,
        remove_env: list[str] | None = None,
    ) -> LaunchLoadState:
        """
        Create the load state of one launch file. The file content is read once
        per file, the node cursor and the include scan position start fresh.
        """
        # normalize, so that cache keys, observer invalidation and the paths
        # reported to the GUI match
        normalized = normalize_path(current_file) if current_file else current_file
        return LaunchLoadState(
            current_file=normalized,
            file_content=self._read_launch_file(normalized) if normalized else "",
            launch_description=(launch_description if launch_description is not None else self.__launch_description),
            indent=indent,
            depth=depth,
            launch_file_obj=launch_file_obj,
            timer_period=timer_period,
            # copies: a nested level must not modify the environment of the caller
            environment=dict(env or {}),
            remove_environment=list(remove_env or []),
        )

    def _create_root_state(self) -> LaunchLoadState:
        """Load state of the top level launch file."""
        return self._create_state_for_file(
            self.filename,
            launch_description=self.__launch_description,
            indent="",
            depth=-1,
            launch_file_obj=None,
            timer_period=0,
        )

    def _cancel_run_timers(self) -> None:
        """Cancel all pending timers created by run_node()."""
        with self._run_timers_lock:
            timers = list(self._run_timers)
            self._run_timers.clear()
        for timer in timers:
            if timer.is_alive():
                Log.debug(f"{self.__class__.__name__}: cancel pending start timer")
                timer.cancel()

    def _reset_loaded_state(self) -> None:
        """Drop everything produced by _load(); keeps the provided launch arguments."""
        self._nodes.clear()
        self._included_files.clear()
        # exceptions belong to the parsed content, therefore they are dropped
        # together with the nodes
        self.load_exceptions.clear()

    def unload(self):
        Log.info(f"unload launch file: {self.filename}")
        # pending delayed starts must not fire for an unloaded configuration
        self._cancel_run_timers()
        self._reset_loaded_state()
        self.launch_arguments.clear()

    def load(self) -> None:
        Log.info(
            f"load launch file: {self.filename}, arguments: {[f'{v.name}:={v.value}' for v in self.launch_arguments]}"
        )
        with LaunchConfig._LOAD_LOCK:
            # reload must not append to the result of a previous load
            self._reset_loaded_state()
            self.environment = os.environ.copy()
            try:
                self._load()
            finally:
                # always restore environment, also on load errors, to avoid
                # interaction if multiple files are loaded
                os.environ.clear()
                os.environ.update(self.environment)
        self.changed = True

    @staticmethod
    def _get_entities(sub_obj) -> list | None:
        """Extract the sub entities of any launch object."""
        if hasattr(sub_obj, "get_sub_entities"):
            return sub_obj.get_sub_entities()
        if hasattr(sub_obj, "entities"):
            return sub_obj.entities
        return sub_obj

    @classmethod
    def _resolve_handler_name(cls, entity_type: type) -> str:
        """Handler method name for an entity type, result is cached per type."""
        cached = cls._HANDLER_NAME_CACHE.get(entity_type)
        if cached is not None:
            return cached
        name = "_handle_unknown_entity"
        for candidate_type, method_name in cls._ENTITY_HANDLERS:
            if issubclass(entity_type, candidate_type):
                name = method_name
                break
        else:
            if hasattr(entity_type, "execute"):
                name = "_handle_executable_entity"
        cls._HANDLER_NAME_CACHE[entity_type] = name
        return name

    def _load(
        self, sub_obj: None | list[launch.frontend.Entity] = None, *, state: LaunchLoadState | None = None
    ) -> None:
        """
        Walk the launch tree and dispatch every entity to its handler.

        :param sub_obj: entities to walk, None for the top level description
        :param state: load state of the current file, created for the root call
        """
        if state is None:
            state = self._create_root_state()
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {state.indent}perform file {state.current_file}")
        if sub_obj is None:
            sub_obj = self.__launch_description
            self.context.extend_locals({"launch_file_path": self.filename})
            self.context.extend_locals({"launch_arguments": self.launch_arguments})
            self.context.extend_locals({v.name: v.value for v in self.launch_arguments})
        if state.current_file:
            self.context.extend_locals({"current_launch_file_path": state.current_file})
        entities = self._get_entities(sub_obj)
        if entities is None:
            return
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {state.indent}entities: {entities}")
        for entity in entities:
            if PRINT_DEBUG_LOAD:
                print(f"  ***debug launch loading: {state.indent}perform entity: {entity}")
            if self._skip_excluded_entity(entity, state):
                continue
            handler_name = self._resolve_handler_name(type(entity))
            getattr(self, handler_name)(entity, state)

    def _skip_excluded_entity(self, entity, state: LaunchLoadState) -> bool:
        """
        Handle entities whose condition does not evaluate to True.

        Even excluded entities have to be parsed, because
          1. included files: the line of the include directive is tracked
          2. GroupActions: the line number is needed for the recursive _load()

        :return: True if the entity must be skipped
        """
        if not (hasattr(entity, "condition") and entity.condition):
            return False
        if entity.condition.evaluate(self.context):
            return False
        if isinstance(entity, launch.actions.include_launch_description.IncludeLaunchDescription):
            # perform search
            inc_file_exists = False
            file_size = -1
            cfg_actions = entity.execute(self.context)
            inc_launch_arguments = []
            if cfg_actions is not None:
                for cac in cfg_actions:
                    if isinstance(cac, launch.actions.set_launch_configuration.SetLaunchConfiguration):
                        cac.execute(self.context)
                        arg_name = perform_substitutions(self.context, cac.name)
                        arg_value = perform_substitutions(self.context, cac.value)
                        if PRINT_DEBUG_LOAD:
                            print(
                                f"  ***debug launch loading: {state.indent}  add launch config: {arg_name}: {arg_value}"
                            )
                        inc_launch_arguments.append(LaunchArgument(name=arg_name, value=arg_value))
            inc_file_name = perform_to_string(self.context, entity.launch_description_source.location)
            # normalize, so that cache keys and observer invalidation match
            used_path = normalize_path(inc_file_name) if inc_file_name else inc_file_name
            used_realpath = used_path
            if used_path and os.path.exists(used_path):
                inc_file_exists = True
                file_size = os.path.getsize(used_path)
                used_realpath = os.path.realpath(used_path)
            # write back, so that the next include is searched further down
            include_line_number, raw_text = self._next_include(state)
            launch_inc_file = LaunchIncludedFile(
                path=state.current_file,
                line_number=include_line_number,
                inc_path=used_path,
                inc_realpath=used_realpath,
                exists=inc_file_exists,
                raw_inc_path=raw_text,
                rec_depth=state.depth + 1,
                args=[],
                default_inc_args=inc_launch_arguments,
                size=file_size,
                conditional_excluded=True,
            )
            self._included_files.append(launch_inc_file)
        elif isinstance(entity, launch.actions.group_action.GroupAction):
            # excluded group: consume the group and all directives inside it,
            # they are never executed
            state.includes().next(GROUP_KIND, skip_content=True)
        return True

    def _raw_name_attribute(self, entity) -> str | None:
        """
        Return the unresolved 'name' attribute of an XML entity, if available.
        """
        element = getattr(entity, "_Entity__xml_element", None)
        return element.get("name") if element is not None else None

    # -- entity handlers -----------------------------------------------------

    def _handle_node(self, entity, state: LaunchLoadState) -> None:
        try:
            if PRINT_DEBUG_LOAD:
                print(f"  ***debug launch loading: {state.indent}  parse node: {entity._Node__node_executable}")
            entity._perform_substitutions(self.context)
            if PRINT_DEBUG_LOAD:
                print(f"  ***debug launch loading: {state.indent}  node after subst: {entity._Node__node_executable}")
            node = LaunchNodeWrapper(entity, load_state=state, launch_context=self.context)
            self._nodes.append(node)
            if isinstance(entity, launch_ros.actions.ComposableNodeContainer):
                for cn in entity._ComposableNodeContainer__composable_node_descriptions:
                    c_node = LaunchNodeWrapper(
                        cn, load_state=state, launch_context=self.context, composable_container=node.unique_name
                    )
                    self._nodes.append(c_node)
        except (SubstitutionFailure, PackageNotFoundError) as err:
            raise err
        except Exception:
            import traceback

            print(traceback.format_exc())

    def _handle_load_composable_nodes(self, entity, state: LaunchLoadState) -> None:
        if PRINT_DEBUG_LOAD:
            print(
                f"  ***debug launch loading: {state.indent}  load composable nodes: "
                f"{len(entity._LoadComposableNodes__composable_node_descriptions)}"
            )
        for cn in entity._LoadComposableNodes__composable_node_descriptions:
            container_name = ""
            if isinstance(entity._LoadComposableNodes__target_container, launch_ros.actions.ComposableNodeContainer):
                node = LaunchNodeWrapper(
                    entity._LoadComposableNodes__target_container, load_state=state, launch_context=self.context
                )
                self._nodes.append(node)
                container_name = node.node_name
            else:
                subs = normalize_to_list_of_substitutions(entity._LoadComposableNodes__target_container)
                container_name = make_namespace_absolute(perform_substitutions(self.context, subs))
            for n in self._nodes:
                if n.node_name == container_name:
                    n.composable_container = container_name
            node = LaunchNodeWrapper(
                cn, load_state=state, launch_context=self.context, composable_container=container_name
            )
            self._nodes.append(node)

    def _handle_execute_process(self, entity, state: LaunchLoadState) -> None:
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {state.indent}  add execute process")
        node = LaunchNodeWrapper(entity, load_state=state, launch_context=self.context)
        self._nodes.append(node)

    def _handle_declare_launch_argument(self, entity, state: LaunchLoadState) -> None:
        entity.execute(self.context)

    def _handle_include_launch_description(self, entity, state: LaunchLoadState) -> None:
        self.context._push_launch_configurations()
        saved_env = dict(os.environ)  # save environment
        try:
            cfg_actions = entity.execute(self.context)
            if PRINT_DEBUG_LOAD:
                print(
                    f"  ***debug launch loading: {state.indent} include file: "
                    f"{entity.launch_description_source.location}"
                )
            inc_file_exists = False
            file_size = -1
            location = entity.launch_description_source.location
            if not isinstance(location, str):
                # after execute() the location is normally a plain string, be
                # defensive for launch description sources with substitutions
                location = perform_to_string(self.context, location)
            # normalize, so that cache keys and observer invalidation match the
            # paths reported to the GUI
            used_path = normalize_path(location) if location else location
            used_realpath = used_path
            if used_path and os.path.exists(used_path):
                inc_file_exists = True
                used_realpath = os.path.realpath(used_path)
                file_size = os.path.getsize(used_path)
            # write back, so that the next include of this file is found behind
            # the current include directive
            include_line_number, raw_text = self._next_include(state)
            if cfg_actions is not None:
                for cac in cfg_actions:
                    if isinstance(cac, launch.actions.set_launch_configuration.SetLaunchConfiguration):
                        cac.execute(self.context)
            launch_inc_file = LaunchIncludedFile(
                path=state.current_file,
                line_number=include_line_number,
                inc_path=used_path,
                inc_realpath=used_realpath,
                exists=inc_file_exists,
                raw_inc_path=raw_text,
                rec_depth=state.depth + 1,
                args=[],
                default_inc_args=[],
                size=file_size,
            )
            self._included_files.append(launch_inc_file)
            included_file = entity._get_launch_file()
            # new file: fresh node cursor and fresh scan position
            self._load(
                entity,
                state=self._create_state_for_file(
                    included_file,
                    launch_description=entity,
                    indent=state.indent + "  ",
                    depth=state.depth + 1,
                    launch_file_obj=launch_inc_file,
                    timer_period=state.timer_period,
                    env=state.environment,
                    remove_env=state.remove_environment,
                ),
            )
            if state.current_file:
                self.context.extend_locals({"current_launch_file_path": state.current_file})
        except launch.invalid_launch_file_error.InvalidLaunchFileError as err:
            raise Exception("%s (%s)" % (err, entity.launch_description_source.location))
        finally:
            os.environ.clear()  # restore environment
            os.environ.update(saved_env)
            self.context._pop_launch_configurations()

    def _handle_group_action(self, entity, state: LaunchLoadState) -> None:
        self.context._push_launch_configurations()
        saved_env = dict(os.environ)  # save environment
        try:
            if state.current_file:
                self.context.extend_locals({"current_launch_file_path": state.current_file})
            # move behind the opening bracket only, the includes inside the
            # consume the group itself, the includes inside are located by the
            # recursion
            state.includes().next(GROUP_KIND)
            # same file: node cursor and scan position are shared
            self._load(entity, state=state.child_same_file())
        finally:
            os.environ.clear()  # restore environment
            os.environ.update(saved_env)
            self.context._pop_launch_configurations()

    def _handle_timer_action(self, entity, state: LaunchLoadState) -> None:
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {state.indent} timer period: {entity.period}")
        period = entity.period
        if not isinstance(period, (float, int)):
            period = float(perform_substitutions(self.context, entity.period))
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {state.indent} timer period (resolved): {period}")
            print(f"  ***debug launch loading: {state.indent} actions count: {len(entity.actions)}")
        self._load(entity.actions, state=state.child_same_file(timer_period=period))

    def _handle_set_environment_variable(self, entity, state: LaunchLoadState) -> None:
        # apply to the real process environment, so that OpaqueFunctions reading
        # os.environ see the same value as with "ros2 launch"
        entity.execute(self.context)
        name = perform_substitutions(self.context, getattr(entity, "name", ""))
        value = perform_substitutions(self.context, getattr(entity, "value", ""))
        state.environment[name] = value
        if name in state.remove_environment:
            state.remove_environment.remove(name)

    def _handle_unset_environment_variable(self, entity, state: LaunchLoadState) -> None:
        if hasattr(entity, "execute"):
            entity.execute(self.context)
        name = perform_substitutions(self.context, getattr(entity, "name", ""))
        if name in state.environment:
            del state.environment[name]
        if name not in state.remove_environment:
            state.remove_environment.append(name)

    def _handle_launch_description(self, entity, state: LaunchLoadState) -> None:
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {state.indent} LaunchDescription: {entity}: {dir(entity)}")
            print(f"  current file: {state.current_file}")
        # guard against an empty list: a top level LaunchDescription can appear
        # before any include was tracked.
        # Both sides of the comparison are normalized paths.
        last_included_file = self._included_files[-1] if self._included_files else None
        if last_included_file and last_included_file.inc_path == state.current_file:
            launch_args = []
            for arg_name, arg_value in state.launch_description.launch_arguments:
                try:
                    la = LaunchArgument(
                        perform_to_string(self.context, arg_name), perform_to_string(self.context, arg_value)
                    )
                    if PRINT_DEBUG_LOAD:
                        print(f"    add launch arg: {la.name}: {la.value}")
                    launch_args.append(la)
                except Exception as err:
                    import traceback

                    print(traceback.format_exc())
                    raise LaunchConfigException(f"Error while resolve arguments in {state.current_file}: {err}")
            default_args = []
            for arg in entity.get_launch_arguments():
                try:
                    da = LaunchArgument(
                        perform_to_string(self.context, arg.name), perform_to_string(self.context, arg.default_value)
                    )
                    if PRINT_DEBUG_LOAD:
                        print(f"    add default arg: {da.name}: {da.value}")
                    default_args.append(da)
                except Exception as err:
                    import traceback

                    print(traceback.format_exc())
                    raise LaunchConfigException(f"Error while resolve default arguments in {state.current_file}: {err}")
            last_included_file.args = launch_args
            last_included_file.default_inc_args = default_args
        # state.position_in_file = self._load(entity, launch_description=state.launch_description,
        self._load(entity, state=state.child_same_file(depth=state.depth + 1))
        if state.current_file:
            self.context.extend_locals({"current_launch_file_path": state.current_file})

    def _handle_set_launch_configuration(self, entity, state: LaunchLoadState) -> None:
        entity.execute(self.context)
        # launch prefix is set by <let ... /> in a group
        arg_name = perform_substitutions(self.context, entity.name)
        if arg_name == "launch-prefix":
            state.launch_prefix = perform_substitutions(self.context, getattr(entity, "value", ""))
        if PRINT_DEBUG_LOAD:
            arg_value = perform_substitutions(self.context, entity.value)
            print(f"  ***debug launch loading: {state.indent}  SetLaunchConfiguration: {arg_name}: {arg_value}")

    def _handle_pop_launch_configurations(self, entity, state: LaunchLoadState) -> None:
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {state.indent}  PopLaunchConfigurations: {dir(entity)}")
        entity.execute(self.context)
        if PRINT_DEBUG_LOAD:
            for sub_entity in entity.get_sub_entities():
                arg_name = perform_substitutions(self.context, sub_entity.name)
                print(f"  ***debug launch loading: {state.indent}    remove arg_name: {arg_name}")

    def _handle_executable_entity(self, entity, state: LaunchLoadState) -> None:
        """Fallback for every entity that provides execute(), e.g. OpaqueFunction."""
        if PRINT_DEBUG_LOAD:
            print(f"  ***debug launch loading: {state.indent} parse execute entity: {entity}; {dir(entity)}")
        try:
            exec_result = entity.execute(self.context)
            if exec_result:
                if PRINT_DEBUG_LOAD:
                    print(f"  ***debug execute result: {exec_result}; {dir(exec_result)}")
                if not isinstance(exec_result, list):
                    exec_result = [exec_result]
                self._load(exec_result, state=state.child_same_file())
        except Exception as e:
            import traceback

            err_msg = traceback.format_exc()
            self.load_exceptions.append(f"{e}")
            print(err_msg)

    def _handle_unknown_entity(self, entity, state: LaunchLoadState) -> None:
        """Fallback for entities without execute(): try to recurse into them."""
        Log.debug(f"{self.__class__.__name__}: {state.indent}unknown entity: {entity}")
        self._load(entity, state=state.child_same_file(depth=state.depth + 1))
        if state.current_file:
            self.context.extend_locals({"current_launch_file_path": state.current_file})

    # -- accessors -----------------------------------------------------------

    def nodes(self) -> list[LaunchNodeWrapper]:
        return self._nodes

    @property
    def filename(self) -> str:
        """
        Returns an existing path with file name or an empty string.
        """
        if os.path.isfile(self.__launch_file):
            return self.__launch_file
        raise LaunchConfigException(f"launch file {self.__launch_file} not found!")

    @property
    def launch_name(self) -> str:
        """
        Returns the name of the launch file with extension, e.g. 'test.launch'
        """
        return os.path.basename(self.__launch_file)

    @property
    def package_name(self) -> str | None:
        """
        Returns the name of the package containing the launch file or None.
        """
        return self.__package

    @classmethod
    def _get_launch_arguments(
        cls, context: LaunchContext, filename: str, *, provided_args: list | None
    ) -> list[LaunchArgument]:
        """
        Returns only top-level launch arguments, but collects all possible default values recursively
        from entire launch tree (includes, groups, nested LaunchDescriptions).

        Note: this executes OpaqueFunctions of the launch file, therefore it must
        only be called with _LOAD_LOCK held and with a context that may be
        modified (see get_launch_arguments()).
        """

        def get_entities(entity):
            """Helper: Extract entities from any entity type."""
            try:
                if hasattr(entity, "entities"):
                    return entity.entities
                elif hasattr(entity, "get_sub_entities"):
                    return entity.get_sub_entities()
                elif hasattr(entity, "describe_sub_entities"):
                    return entity.describe_sub_entities()
            except Exception:
                pass
            return []

        def collect_all_defaults(entities, defaults_map: defaultdict):
            """Recursively collect ALL DeclareLaunchArgument defaults from entire launch tree."""
            for entity in entities:
                # Direct DeclareLaunchArgument
                if isinstance(entity, launch.actions.DeclareLaunchArgument):
                    if entity.default_value:
                        try:
                            resolved = launch.utilities.perform_substitutions(context, entity.default_value)
                            if resolved not in defaults_map[entity.name]:
                                defaults_map[entity.name].append(resolved)
                        except Exception:
                            pass

                # IncludeLaunchDescription
                elif isinstance(entity, launch.actions.IncludeLaunchDescription):
                    sub_entities = entity.describe_sub_entities()
                    collect_all_defaults(sub_entities, defaults_map)
                    for _, cond_entities in entity.describe_conditional_sub_entities():
                        collect_all_defaults(cond_entities, defaults_map)

                # GroupAction
                elif isinstance(entity, launch.actions.GroupAction):
                    collect_all_defaults(get_entities(entity), defaults_map)

                # LaunchDescription (NESTED!)
                elif isinstance(entity, launch.LaunchDescription):
                    collect_all_defaults(entity.entities, defaults_map)

                # OpaqueFunction
                elif isinstance(entity, launch.actions.OpaqueFunction):
                    try:
                        result = entity.execute(context)
                        if isinstance(result, list):
                            for sub_result in result:
                                collect_all_defaults([sub_result], defaults_map)
                        else:
                            collect_all_defaults([result], defaults_map)
                    except Exception:
                        pass

                # Generic recursion for other containers
                else:
                    collect_all_defaults(get_entities(entity), defaults_map)

        launch_description = get_launch_description_from_any_launch_file(filename)

        # Phase 1: ONLY top-level DeclareLaunchArgument (direct from launch_description.entities)
        top_level_actions = []
        for entity in launch_description.entities:
            if isinstance(entity, launch.actions.DeclareLaunchArgument):
                if entity.name not in [e.name for e in top_level_actions]:
                    top_level_actions.append(entity)

        # Phase 2: FULL recursion - collect ALL defaults from entire tree
        all_defaults_map = defaultdict(list)
        collect_all_defaults(launch_description.entities, all_defaults_map)

        # Build result
        result = []
        for argument_action in top_level_actions:
            value = None
            if provided_args is not None:
                for provided_arg in provided_args:
                    if argument_action.name == provided_arg.name and hasattr(provided_arg, "value"):
                        value = provided_arg.value
                        break

            default_value = None
            if argument_action.default_value:
                default_value = launch.utilities.perform_substitutions(context, argument_action.default_value)

            # Extend choices with ALL found defaults
            all_choices = list(all_defaults_map[argument_action.name])
            existing_choices = argument_action.choices or []
            choices = list(existing_choices)
            for default_val in all_choices:
                if default_val not in choices:
                    choices.append(default_val)

            arg = LaunchArgument(
                name=argument_action.name,
                value=value if value is not None else default_value,
                default_value=default_value,
                description=argument_action.description,
                choices=choices,
            )
            result.append(arg)

        return result

    @classmethod
    def get_launch_arguments(
        cls, context: LaunchContext, filename: str, *, provided_args: list | None
    ) -> list[LaunchArgument]:
        """
        Cached top-level launch arguments of a launch file.

        The parse executes OpaqueFunctions of the launch file, which can modify
        os.environ and the launch configurations of the used context. Therefore
        an own context is created for the parse and the parse itself runs with
        _LOAD_LOCK held, with a restored environment afterwards. The 'context'
        parameter is kept for API compatibility and is only used as a fallback
        source for the argv of the parse context.
        """
        normalized_filename = normalize_path(filename)

        # Own context for the parse: executing entities must not leak launch
        # configurations into the caller's context.
        parse_argv = list(getattr(context, "argv", None) or sys.argv[1:])
        parse_context = LaunchContext(argv=parse_argv)

        # The key is built from the context that is actually used for the parse,
        # so key and cached result always belong together.
        key = LAUNCH_ARGUMENT_CACHE.make_key(normalized_filename, parse_context)

        def _load_templates() -> tuple[LaunchArgTemplate, ...]:
            Log.debug(f"LaunchConfig: parse launch arguments of {normalized_filename}")

            # Entity execution touches os.environ, serialize it like load()
            # and always restore the environment.
            with cls._LOAD_LOCK:
                saved_env = dict(os.environ)
                try:
                    # The cached result must not contain request-specific values.
                    parsed = cls._get_launch_arguments(parse_context, filename=normalized_filename, provided_args=None)
                finally:
                    os.environ.clear()
                    os.environ.update(saved_env)

            return tuple(
                LaunchArgTemplate(
                    name=argument.name,
                    default_value=argument.default_value,
                    description=argument.description,
                    choices=tuple(argument.choices or []),
                )
                for argument in parsed
            )

        templates = LAUNCH_ARGUMENT_CACHE.get_or_load(key, _load_templates)

        # Always return fresh request-specific objects.
        provided = {}

        if provided_args:
            provided = {argument.name: argument.value for argument in provided_args if hasattr(argument, "value")}

        return [
            LaunchArgument(
                name=template.name,
                value=provided.get(template.name, template.default_value),
                default_value=template.default_value,
                description=template.description,
                choices=list(template.choices),
            )
            for template in templates
        ]

    def get_node(self, name: str) -> LaunchNodeWrapper | None:
        """
        Returns a configuration node for a given node name.
        """
        for item in self.nodes():
            if item.unique_name == name:
                return item
        Log.debug(f"Node '{name}' NOT found; {self.filename}; nodes: {len(self._nodes)}")
        return None

    def run_node(
        self, name: str, ignore_timer=False, *, executable_callback: Callable[[str, str], None] | None = None
    ) -> str:
        """
        Start a node local

        :param str name: unique name of the node to start.
        :param bool ignore_timer: start immediately, even if the node was defined
                                  inside a TimerAction.
        :param executable_callback: optional callback, called with
                                    (node name, executable path) as soon as the
                                    executable is known. It is also called for
                                    delayed (timer) starts, so that the caller
                                    can register the executable at the file
                                    observer. Composed nodes have no executable
                                    and do not trigger the callback.
        :return: path of executable or empty string on load composable node or
                 on delayed start.
        :raise exceptions.StartException: on errors
        :raise exceptions.BinarySelectionRequest: on multiple binaries
        """
        node: LaunchNodeWrapper = self.get_node(name)
        if node is None:
            raise exceptions.StartException(f"Node '{name}' in '{self.filename}' not found!")
        if node.timer_period > 0 and not ignore_timer:
            # the executable is only known after the delayed start, therefore the
            # caller is informed by the callback (observer registration)
            timer = threading.Timer(node.timer_period, self._run_node_delayed, args=(name, executable_callback))
            timer.daemon = True
            with self._run_timers_lock:
                # drop finished timers, the list must not grow forever
                self._run_timers = [t for t in self._run_timers if t.is_alive()]
                self._run_timers.append(timer)
            timer.start()
            return f"{name} will be started in {node.timer_period} seconds"
        if node.composable_container and node.composable_container != node.node_name:
            # load plugin in container
            Log.info(f"Load node='{node.unique_name}'; as plugin into container='{node.composable_container}';")
            # skip check if container is running, it is done by the GUI
            self.run_composed_node(node)
            return ""

        # run on local host
        # run get_cmd() before create new_env since get_cmd() extends os.environ
        with LaunchConfig._LOAD_LOCK:
            screen_prefix = screen.get_cmd(node.unique_name)
            # set environment
            new_env = os.environ.copy()
        # add environment from launch
        if node.additional_env:
            new_env.update(dict(node.additional_env))
        for env_name in node.remove_environment:
            if env_name in new_env:
                del new_env[env_name]
        # set display variable to local display
        if "DISPLAY" in new_env:
            if not new_env["DISPLAY"] or new_env["DISPLAY"] == "remote":
                del new_env["DISPLAY"]
        else:
            new_env["DISPLAY"] = ":0"
        if node.node_namespace:
            new_env["ROS_NAMESPACE"] = node.node_namespace
        # set logging
        if node.output_format:
            new_env["ROSCONSOLE_FORMAT"] = "%s" % node.output_format
        # handle respawn
        respawn_prefix = ""
        if node.respawn:
            if node.respawn_delay and node.respawn_delay > 0:
                new_env["RESPAWN_DELAY"] = "%d" % node.respawn_delay
            respawn_prefix = f"{RESPAWN_SCRIPT}"

        launch_prefix = ""
        if node.launch_prefix:
            launch_prefix = node.launch_prefix
        # TODO: check for HOSTNAME
        # start
        executable_path = ""
        if node.cmd:
            executable_path = node.cmd.split()[0]
        Log.info(f"{screen_prefix} {respawn_prefix} {launch_prefix} {node.cmd} (launch_file: '{node.launch_name}')")
        Log.debug(f"environment while run node '{node.unique_name}': '{new_env}'")
        SupervisedPopen(
            " ".join([screen_prefix, respawn_prefix, launch_prefix, node.cmd]),
            cwd=node.cwd,
            shell=True,
            env=new_env,
            object_id=f"run_node_{node.unique_name}",
            description=f"Run [{node.package_name}]{node.executable}",
        )
        if executable_callback is not None and executable_path:
            try:
                executable_callback(node.unique_name, executable_path)
            except Exception as error:
                Log.warn(f"failed to report executable of '{node.unique_name}': {error}")
        return executable_path

    def _run_node_delayed(self, name: str, executable_callback: Callable[[str, str], None] | None) -> None:
        """
        Timer target for nodes defined inside a TimerAction. Errors must not
        escape into the timer thread, they are only logged here.
        """
        try:
            self.run_node(name, ignore_timer=True, executable_callback=executable_callback)
        except Exception as error:
            Log.error(f"failed to start delayed node '{name}' of '{self.launch_name}': {error}")

    def run_composed_node(self, node: LaunchNodeWrapper):
        request = node.get_composed_load_request()
        service_load_node_name = f"{node.composable_container}/_container/load_node"
        Log.debug(f" -> load composed node to '{service_load_node_name}'")
        response = nmd.launcher.call_service(
            service_load_node_name, composition_interfaces.srv.LoadNode, request, timeout_sec=COMPONENT_LOAD_TIMEOUT
        )
        if response is None:
            error_msg = f"Failed to load service '{request.node_name}' of type '{request.plugin_name}' in container '{node.composable_container}': None as service response"
            Log.error(error_msg)
            raise exceptions.StartException(error_msg)
        Log.debug(f"  <- load composed node: response received: {response} {dir(response)}")
        node_name = response.full_node_name if response.full_node_name else request.node_name
        if response.success:
            Log.info(f"Loaded node '{response.full_node_name}' in container '{node.composable_container}'")
        else:
            error_msg = f"Failed to load node '{node_name}' of type '{request.plugin_name}' in container '{node.composable_container}': {response.error_message}"
            Log.error(error_msg)
            raise exceptions.StartException(error_msg)
