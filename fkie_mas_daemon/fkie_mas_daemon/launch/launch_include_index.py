# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import ast
import os
import re
from dataclasses import dataclass
from xml.parsers import expat

import yaml
from fkie_mas_pylib.logging.logging import Log

INCLUDE_KIND = "include"
GROUP_KIND = "group"

# python identifiers that introduce an include or a group
PYTHON_INCLUDE_CALLS = ("IncludeLaunchDescription",)
PYTHON_GROUP_CALLS = ("GroupAction",)
# wrappers whose first argument holds the raw path of an include
PYTHON_SOURCE_CALLS = (
    "PythonLaunchDescriptionSource",
    "XMLLaunchDescriptionSource",
    "YAMLLaunchDescriptionSource",
    "AnyLaunchDescriptionSource",
    "FrontendLaunchDescriptionSource",
)

_XML_FALLBACK_RE = re.compile(r"<(include|group)\b([^>]*)>", re.IGNORECASE)
_XML_FILE_ATTR_RE = re.compile(r'\bfile\s*=\s*"([^"]*)"|\bfile\s*=\s*\'([^\']*)\'')
_PY_FALLBACK_RE = re.compile(r"\b(IncludeLaunchDescription|GroupAction)\s*\(")
_LOOP_NODES = (ast.For, ast.AsyncFor, ast.While, ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)


@dataclass
class IncludeDefinition:
    """
    One include or group directive as it appears in the launch file text.
    """

    kind: str  # INCLUDE_KIND or GROUP_KIND
    line_number: int  # 1-based line of the directive
    offset: int  # character offset of the directive
    end_offset: int  # character offset behind the whole element
    raw_path: str = ""  # unresolved path expression, '' for groups
    # directive inside a loop: it can be executed more than once
    repeatable: bool = False


# -- helper ------------------------------------------------------------------


def _line_starts(content: str) -> list[int]:
    starts = [0]
    for match in re.finditer("\n", content):
        starts.append(match.end())
    return starts


def _offset_of(line_starts: list[int], line: int, column: int) -> int:
    """Character offset of a 1-based line and a 0-based column."""
    index = line - 1
    if index < 0 or index >= len(line_starts):
        return 0
    return line_starts[index] + column


def _line_of(line_starts: list[int], offset: int) -> int:
    low, high = 0, len(line_starts) - 1
    while low < high:
        middle = (low + high + 1) // 2
        if line_starts[middle] <= offset:
            low = middle
        else:
            high = middle - 1
    return low + 1


def _mask_xml_comments(content: str) -> str:
    """
    Replace comment content by spaces, keeping offsets and line numbers.
    Only used by the regex fallback, the real parsers ignore comments anyway.
    """

    def replace(match: "re.Match") -> str:
        return "".join(char if char == "\n" else " " for char in match.group(0))

    return re.sub(r"<!--.*?-->", replace, content, flags=re.DOTALL)


# -- parser ------------------------------------------------------------------


def _parse_xml(content: str) -> list[IncludeDefinition]:
    """
    Parse with expat: comments, CDATA and attribute quoting are handled by the
    parser, so no text preprocessing is needed.
    """
    line_starts = _line_starts(content)
    definitions: list[IncludeDefinition] = []
    open_stack: list[IncludeDefinition] = []

    def start_element(name: str, attrs: dict[str, str]) -> None:
        if name not in (INCLUDE_KIND, GROUP_KIND):
            return
        definition = IncludeDefinition(
            kind=name,
            line_number=parser.CurrentLineNumber,
            offset=_offset_of(line_starts, parser.CurrentLineNumber, parser.CurrentColumnNumber),
            end_offset=len(content),
            raw_path=attrs.get("file", "") if name == INCLUDE_KIND else "",
        )
        definitions.append(definition)
        open_stack.append(definition)

    def end_element(name: str) -> None:
        if name not in (INCLUDE_KIND, GROUP_KIND) or not open_stack:
            return
        definition = open_stack.pop()
        definition.end_offset = _offset_of(line_starts, parser.CurrentLineNumber, parser.CurrentColumnNumber)

    parser = expat.ParserCreate()
    parser.StartElementHandler = start_element
    parser.EndElementHandler = end_element
    try:
        parser.Parse(content, True)
    except expat.ExpatError as error:
        # the file may be edited in the GUI and temporarily invalid
        Log.debug(f"launch include index: xml parse failed ({error}), use fallback")
        return _parse_xml_fallback(content)
    return definitions


def _parse_xml_fallback(content: str) -> list[IncludeDefinition]:
    masked = _mask_xml_comments(content)
    line_starts = _line_starts(masked)
    definitions: list[IncludeDefinition] = []
    for match in _XML_FALLBACK_RE.finditer(masked):
        kind = match.group(1).lower()
        raw_path = ""
        if kind == INCLUDE_KIND:
            attr = _XML_FILE_ATTR_RE.search(match.group(2) or "")
            if attr is not None:
                raw_path = attr.group(1) or attr.group(2) or ""
        definitions.append(
            IncludeDefinition(
                kind=kind,
                line_number=_line_of(line_starts, match.start()),
                offset=match.start(),
                # element end is unknown here, so no child directive is skipped
                end_offset=match.end(),
                raw_path=raw_path,
            )
        )
    return definitions


def _call_name(func: ast.AST) -> str:
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return ""


def _raw_include_path(node: ast.Call, content: str) -> str:
    """Source text of the launch description source of an include call."""
    candidates = list(node.args) + [keyword.value for keyword in node.keywords]
    for candidate in candidates:
        if isinstance(candidate, ast.Call) and _call_name(candidate.func) in PYTHON_SOURCE_CALLS:
            if candidate.args:
                return ast.get_source_segment(content, candidate.args[0]) or ""
            return ast.get_source_segment(content, candidate) or ""
    if candidates:
        return ast.get_source_segment(content, candidates[0]) or ""
    return ""


def _parse_python(content: str) -> list[IncludeDefinition]:
    line_starts = _line_starts(content)
    try:
        tree = ast.parse(content)
    except SyntaxError as error:
        Log.debug(f"launch include index: python parse failed ({error}), use fallback")
        return _parse_python_fallback(content)

    loop_spans: list[tuple[int, int]] = [
        (node.lineno, getattr(node, "end_lineno", node.lineno))
        for node in ast.walk(tree)
        if isinstance(node, _LOOP_NODES)
    ]

    definitions: list[IncludeDefinition] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = _call_name(node.func)
        if name in PYTHON_INCLUDE_CALLS:
            kind = INCLUDE_KIND
        elif name in PYTHON_GROUP_CALLS:
            kind = GROUP_KIND
        else:
            continue
        end_line = getattr(node, "end_lineno", node.lineno)
        end_column = getattr(node, "end_col_offset", 0)
        # only includes are marked repeatable: a repeatable group would also
        # repeat its children, which the linear cursor cannot represent
        repeatable = kind == INCLUDE_KIND and any(start <= node.lineno and end_line <= end for start, end in loop_spans)
        definitions.append(
            IncludeDefinition(
                kind=kind,
                line_number=node.lineno,
                offset=_offset_of(line_starts, node.lineno, node.col_offset),
                end_offset=_offset_of(line_starts, end_line, end_column),
                raw_path=_raw_include_path(node, content) if kind == INCLUDE_KIND else "",
                repeatable=repeatable,
            )
        )
    definitions.sort(key=lambda item: (item.line_number, item.offset))
    return definitions


def _parse_python_fallback(content: str) -> list[IncludeDefinition]:
    line_starts = _line_starts(content)
    definitions: list[IncludeDefinition] = []
    for match in _PY_FALLBACK_RE.finditer(content):
        kind = INCLUDE_KIND if match.group(1) in PYTHON_INCLUDE_CALLS else GROUP_KIND
        definitions.append(
            IncludeDefinition(
                kind=kind,
                line_number=_line_of(line_starts, match.start()),
                offset=match.start(),
                end_offset=match.end(),
            )
        )
    return definitions


def _yaml_file_value(node: "yaml.Node") -> str:
    if isinstance(node, yaml.MappingNode):
        for key, value in node.value:
            if getattr(key, "value", None) == "file":
                return str(getattr(value, "value", ""))
    return str(getattr(node, "value", ""))


def _parse_yaml(content: str) -> list[IncludeDefinition]:
    try:
        root = yaml.compose(content)
    except yaml.YAMLError as error:
        Log.debug(f"launch include index: yaml parse failed ({error}), no includes")
        return []
    definitions: list[IncludeDefinition] = []

    def walk(node: "yaml.Node") -> None:
        if isinstance(node, yaml.MappingNode):
            for key, value in node.value:
                kind = getattr(key, "value", None)
                if kind in (INCLUDE_KIND, GROUP_KIND):
                    definitions.append(
                        IncludeDefinition(
                            kind=kind,
                            line_number=key.start_mark.line + 1,
                            offset=key.start_mark.index,
                            end_offset=value.end_mark.index,
                            raw_path=_yaml_file_value(value) if kind == INCLUDE_KIND else "",
                        )
                    )
                walk(value)
        elif isinstance(node, yaml.SequenceNode):
            for item in node.value:
                walk(item)

    if root is not None:
        walk(root)
    definitions.sort(key=lambda item: (item.line_number, item.offset))
    return definitions


def parse_include_definitions(launch_type: str, content: str) -> list[IncludeDefinition]:
    if not content:
        return []
    if launch_type == "python":
        return _parse_python(content)
    if launch_type == "yaml":
        return _parse_yaml(content)
    return _parse_xml(content)


# -- cache -------------------------------------------------------------------


class IncludeDefinitionCache:
    """
    Caches the include index of a launch file, keyed by path and file stamp.
    Unsaved editor content is cached by its hash.
    """

    def __init__(self) -> None:
        self._entries: dict[tuple, list[IncludeDefinition]] = {}

    def _key(self, path: str, content: str | None) -> tuple:
        if content is not None:
            return (path, "content", hash(content))
        try:
            stat = os.stat(path)
            return (path, stat.st_mtime_ns, stat.st_size)
        except OSError:
            return (path, "missing", 0)

    def get(self, path: str, launch_type: str, content: str | None = None) -> list[IncludeDefinition]:
        key = self._key(path, content)
        definitions = self._entries.get(key)
        if definitions is not None:
            return definitions
        text = content
        if text is None:
            try:
                with open(path) as file_handle:
                    text = file_handle.read()
            except OSError as error:
                Log.debug(f"launch include index: cannot read {path}: {error}")
                text = ""
        definitions = parse_include_definitions(launch_type, text)
        self._entries[key] = definitions
        return definitions

    def invalidate(self, path: str | None = None) -> None:
        if path is None:
            self._entries.clear()
            return
        for key in [key for key in self._entries if key[0] == path]:
            del self._entries[key]


_INCLUDE_CACHE = IncludeDefinitionCache()


def get_include_definitions(path: str, launch_type: str, content: str | None = None) -> list[IncludeDefinition]:
    return _INCLUDE_CACHE.get(path, launch_type, content)


def invalidate_include_definitions(path: str | None = None) -> None:
    _INCLUDE_CACHE.invalidate(path)


# -- cursor ------------------------------------------------------------------


class IncludeDefinitionCursor:
    """
    Forward cursor over the include and group directives of one launch file.

    Includes and groups share one monotone position, exactly like the former
    text scan: locating a group also consumes the directives before it.
    """

    def __init__(self, definitions: list[IncludeDefinition]) -> None:
        self._definitions = definitions
        self._index = 0

    def next(self, kind: str, *, skip_content: bool = False) -> IncludeDefinition | None:
        """
        Return the next directive of the given kind, or None if the file has
        no further one (e.g. includes created dynamically by an
        OpaqueFunction).

        :param skip_content: also consume all directives nested in the
                             returned element, used for excluded groups whose
                             children are never executed
        """
        index = self._index
        while index < len(self._definitions):
            definition = self._definitions[index]
            if definition.kind != kind:
                index += 1
                continue
            if definition.repeatable:
                # may be executed again, keep the position
                self._index = index
            elif skip_content:
                self._index = self._end_of_element(index, definition)
            else:
                self._index = index + 1
            return definition
        self._index = len(self._definitions)
        return None

    def _end_of_element(self, index: int, definition: IncludeDefinition) -> int:
        next_index = index + 1
        while next_index < len(self._definitions) and self._definitions[next_index].offset < definition.end_offset:
            next_index += 1
        return next_index
