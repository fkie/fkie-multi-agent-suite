# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from dataclasses import dataclass, field
from typing import Any

from fkie_mas_pylib.interface.launch_interface import LaunchIncludedFile

from .launch_definition_index import NodeDefinitionCursor, get_node_definitions, launch_type_of
from .launch_include_index import IncludeDefinitionCursor, get_include_definitions


@dataclass
class LaunchLoadState:
    """
    Mutable state of a single LaunchConfig._load() invocation.

    A new instance is created per *file*. Recursions inside the same file use
    child_same_file(), which shares the node definition cursor and the text
    scan position, but copies the environment, so that nothing leaks back to
    the caller. 'launch_prefix' intentionally starts empty on each level, like
    the local variable it replaces.
    """

    current_file: str
    file_content: str
    launch_description: Any
    indent: str
    depth: int
    launch_file_obj: LaunchIncludedFile | None
    timer_period: float
    environment: dict[str, str] = field(default_factory=dict)
    remove_environment: list[str] = field(default_factory=list)
    launch_prefix: str = ""
    # forward cursor over the indexed node definitions of current_file
    definition_cursor: NodeDefinitionCursor | None = None
    # forward cursor over the include/group directives of current_file
    include_cursor: IncludeDefinitionCursor | None = None

    def cursor(self) -> NodeDefinitionCursor:
        """
        Return the node definition cursor, creating it on first use.
        """
        if self.definition_cursor is None:
            self.definition_cursor = NodeDefinitionCursor(
                get_node_definitions(self.current_file, launch_type_of(self.current_file), self.file_content or None)
            )
        return self.definition_cursor

    def includes(self) -> IncludeDefinitionCursor:
        """Return the include cursor, creating it on first use."""
        if self.include_cursor is None:
            self.include_cursor = IncludeDefinitionCursor(
                get_include_definitions(self.current_file, launch_type_of(self.current_file), self.file_content or None)
            )
        return self.include_cursor

    def child_same_file(
        self, *, launch_description: Any = None, timer_period: float | None = None, depth: int | None = None
    ) -> "LaunchLoadState":
        """
        Child state for a recursion that stays inside the same launch file.

        The node definition cursor and the include scan position are shared,
        the environment is copied.
        """
        child = LaunchLoadState(
            current_file=self.current_file,
            file_content=self.file_content,
            launch_description=(self.launch_description if launch_description is None else launch_description),
            indent=self.indent + "  ",
            depth=self.depth if depth is None else depth,
            launch_file_obj=self.launch_file_obj,
            timer_period=(self.timer_period if timer_period is None else timer_period),
            environment=dict(self.environment),
            remove_environment=list(self.remove_environment),
        )
        # share, do not restart at the beginning of the file
        child.definition_cursor = self.cursor()
        child.include_cursor = self.includes()
        return child
