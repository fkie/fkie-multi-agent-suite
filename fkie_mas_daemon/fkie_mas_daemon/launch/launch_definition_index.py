# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import re

from fkie_mas_pylib.logging.logging import Log

from .caches import FILE_CONTENT_CACHE, MISSING, LruTtlCache, stat_fingerprint

# XML tags that can carry a node name.
XML_NODE_TAG = re.compile(r"<\s*(?:node|node_container|composable_node|executable)\b[^>]*", re.S)
XML_NAME_ATTR = re.compile(r'\b(name|exec|executable)\s*=\s*(["\'])(.*?)\2', re.S)

# Python constructors that create a node.
PY_NODE_CALL = re.compile(
    r"\b(?:Node|LifecycleNode|ComposableNode|ComposableNodeContainer"
    r"|ExecuteProcess)\s*\("
)
PY_NAME_KWARG = re.compile(r"\bname\s*=\s*")
PY_STRING_LITERAL = re.compile(r'(["\'])(.*?)\1', re.S)
PY_LOOP_LINE = re.compile(r"^(?:for|while)\b")
PY_SCOPE_LINE = re.compile(r"^(?:def|class)\b")

# Maximum distance between a node constructor and its name keyword.
PY_KWARG_LOOKAHEAD = 2000


def split_substitutions(raw: str) -> list[str]:
    """
    Split a raw attribute value into literal parts and drop '$(...)' blocks.

    Nested parentheses are handled, e.g. '$(eval "$(var a)")'.
    """
    parts: list[str] = []
    literal = ""
    index = 0

    while index < len(raw):
        if raw.startswith("$(", index):
            parts.append(literal)
            literal = ""
            depth = 1
            index += 2

            while index < len(raw) and depth:
                if raw[index] == "(":
                    depth += 1
                elif raw[index] == ")":
                    depth -= 1
                index += 1
        else:
            literal += raw[index]
            index += 1

    parts.append(literal)
    return parts


def attribute_matches(raw: str, resolved: str) -> bool:
    """
    Check whether a raw attribute value can produce the resolved node name.

    A substitution must resolve to something, therefore '.+?' is used between
    the literal parts.
    """
    if raw == resolved:
        return True
    if "$(" not in raw:
        return False

    pattern = ".+?".join(re.escape(part) for part in split_substitutions(raw))
    return re.fullmatch(pattern, resolved, re.S) is not None


def is_wildcard_name(raw: str) -> bool:
    """
    Check whether a raw name can produce any node name.

    True for an empty name (non literal Python keyword) and for names that
    consist of substitutions only, e.g. '$(var container_name)'. Such a
    definition must never win over a definition with literal text.
    """
    if not raw:
        return True
    if "$(" not in raw:
        return False
    return not any(part for part in split_substitutions(raw))


class NodeDefinition:
    """
    Immutable position of one node definition inside a launch file.

    Instances are shared through the process-wide cache and must never be
    modified by callers.
    """

    __slots__ = ("raw_name", "file_range", "offset_end", "repeatable", "wildcard")

    def __init__(
        self, raw_name: str, file_range: dict[str, int], offset_end: int, repeatable: bool, wildcard: bool | None = None
    ) -> None:
        """
        :param raw_name:
            Unresolved name text, may contain '$(...)' or be empty.
        :param file_range:
            Position of the name attribute inside the launch file.
        :param offset_end:
            Character offset directly behind the matched attribute.
        :param repeatable:
            True if this source location can create more than one node, for
            example inside a loop or when the name contains a substitution.
        """
        self.raw_name = raw_name
        self.file_range = file_range
        self.offset_end = offset_end
        self.repeatable = repeatable
        # unspecific definition: matches by position only
        self.wildcard = is_wildcard_name(raw_name) if wildcard is None else wildcard

    def __repr__(self) -> str:
        return (
            f"<NodeDefinition name={self.raw_name!r} "
            f"line={self.file_range['startLineNumber']} "
            f"repeatable={self.repeatable}>"
        )


def _range_of(content: str, start: int, end: int) -> dict[str, int]:
    """
    Convert character offsets into a one-based line and column range.
    """
    start_line_begin = content.rfind("\n", 0, start) + 1
    end_line_begin = content.rfind("\n", 0, end) + 1

    return {
        "startLineNumber": content.count("\n", 0, start) + 1,
        "endLineNumber": content.count("\n", 0, end) + 1,
        "startColumn": start - start_line_begin + 1,
        "endColumn": end - end_line_begin + 1,
    }


def _python_inside_loop(content: str, offset: int) -> bool:
    """
    Check whether an offset lies inside a for or while block.

    The check walks upwards to the first line with a smaller indentation and
    stops at a function or class definition. List comprehensions on the same
    logical line are covered by the ' for ' test.
    """
    line_begin = content.rfind("\n", 0, offset) + 1
    line = content[line_begin : content.find("\n", offset) if content.find("\n", offset) > -1 else len(content)]

    if " for " in line:
        return True

    indent = len(line) - len(line.lstrip())
    cursor = line_begin

    while cursor > 0:
        previous_begin = content.rfind("\n", 0, cursor - 1) + 1
        previous = content[previous_begin : cursor - 1]
        cursor = previous_begin
        stripped = previous.strip()

        if not stripped or stripped.startswith("#"):
            continue

        previous_indent = len(previous) - len(previous.lstrip())

        if previous_indent >= indent:
            continue
        if PY_LOOP_LINE.match(stripped):
            return True
        if PY_SCOPE_LINE.match(stripped):
            return False

        indent = previous_indent

    return False


def _index_xml(content: str) -> list[NodeDefinition]:
    """
    Collect all node definitions of an XML launch file in file order.
    """
    definitions: list[NodeDefinition] = []

    for tag in XML_NODE_TAG.finditer(content):
        attributes = {match.group(1): match for match in XML_NAME_ATTR.finditer(tag.group(0))}
        attribute = attributes.get("name") or attributes.get("exec") or attributes.get("executable")

        if attribute is None:
            continue

        start = tag.start() + attribute.start()
        end = tag.start() + attribute.end()
        raw_name = attribute.group(3)

        definitions.append(NodeDefinition(raw_name, _range_of(content, start, end), end, "$(" in raw_name))

    return definitions


def _index_python(content: str) -> list[NodeDefinition]:
    """
    Collect all node definitions of a Python launch file in file order.

    Names that are not plain string literals, for example lists of
    substitutions or f-strings, are stored as empty raw names and marked as
    repeatable.
    """
    definitions: list[NodeDefinition] = []

    for call in PY_NODE_CALL.finditer(content):
        limit = min(len(content), call.end() + PY_KWARG_LOOKAHEAD)
        kwarg = PY_NAME_KWARG.search(content, call.end(), limit)

        start = call.start()
        end = call.end()
        raw_name = ""
        literal = False

        if kwarg is not None:
            value = PY_STRING_LITERAL.match(content, kwarg.end())
            start = kwarg.start()

            if value is not None:
                end = value.end()
                raw_name = value.group(2)
                literal = True
            else:
                end = kwarg.end()

        definitions.append(
            NodeDefinition(
                raw_name, _range_of(content, start, end), end, not literal or _python_inside_loop(content, call.start())
            )
        )

    return definitions


# Process-wide cache for indexed launch files. No TTL is required because every
# entry is validated with a file fingerprint.
NODE_DEFINITION_CACHE = LruTtlCache(maxsize=256, name="NodeDefinitionCache")


def index_node_definitions(content: str, launch_type: str) -> list[NodeDefinition]:
    """
    Scan launch file content once and return all node definitions.
    """
    if not content:
        return []
    if launch_type == "xml":
        return _index_xml(content)
    return _index_python(content)


def get_node_definitions(file_name: str, launch_type: str, content: str = None) -> list[NodeDefinition]:
    """
    Return the cached node definitions of a launch file.

    The result is validated with a file fingerprint, therefore external file
    changes are detected even without a watchdog event.
    """
    if not file_name:
        return index_node_definitions(content or "", launch_type)

    fingerprint = stat_fingerprint(file_name)
    cache_key = (os.path.realpath(file_name), launch_type)

    def factory() -> list[NodeDefinition]:
        text = content

        if text is None:
            text = FILE_CONTENT_CACHE.get_content(file_name, default=MISSING)

        if text is MISSING:
            Log.debug(f"NodeDefinitionCache: cannot read {file_name}")
            return []

        definitions = index_node_definitions(text, launch_type)
        Log.debug(f"NodeDefinitionCache: indexed {len(definitions)} node definitions in {file_name}")
        return definitions

    if fingerprint is None:
        # The file cannot be stat'ed, so the result must not be cached.
        return factory()

    return NODE_DEFINITION_CACHE.get_or_create(cache_key, factory, fingerprint)


def invalidate_node_definitions(path: str) -> int:
    """
    Invalidate the cached index of one launch file.

    :return:
        Number of removed entries.
    """
    cache_path = os.path.realpath(path)

    return NODE_DEFINITION_CACHE.invalidate_if(
        lambda key: isinstance(key, tuple) and len(key) >= 1 and key[0] == cache_path
    )


def launch_type_of(file_name: str) -> str:
    """
    Return 'xml' for XML or YAML frontend launch files, otherwise 'python'.
    """
    return "xml" if os.path.splitext(file_name)[1].lower() in (".xml", ".launch", ".yaml", ".yml") else "python"


class NodeDefinitionCursor:
    """
    Forward cursor over the node definitions of one launch file.

    The cursor keeps the per-load state that must not be stored in the shared
    cache entries: which resolved names a definition already produced and how
    far the scan advanced.
    """

    # match ranks, lower is better
    _RANK_EXACT = 0
    _RANK_SUBSTITUTION = 1
    _RANK_WILDCARD = 2

    def __init__(self, definitions: list[NodeDefinition]) -> None:
        self._definitions = definitions
        self._served: dict[int, set[str]] = {}
        self._index = 0

    def _literal_match(self, definition: NodeDefinition, resolved: str, raw_name: str | None) -> bool:
        """
        Check whether a definition with a literal name belongs to the node.
        """
        if raw_name is not None and raw_name == definition.raw_name:
            return True
        return attribute_matches(definition.raw_name, resolved)

    def _take(self, index: int, resolved: str) -> NodeDefinition:
        """
        Mark a definition as used for 'resolved' and advance the cursor.
        """
        definition = self._definitions[index]
        self._served.setdefault(index, set()).add(resolved)

        if definition.repeatable:
            # loops and substitutions can create several nodes from one location
            self._index = index
        else:
            self._index = index + 1

        return definition

    def _match_rank(self, definition: NodeDefinition, resolved: str, raw_name: str | None) -> int | None:
        """
        Rank how specific a definition matches the node, None if it cannot.
        """
        if definition.wildcard:
            return self._RANK_WILDCARD
        if raw_name is not None and raw_name == definition.raw_name:
            return self._RANK_EXACT
        if definition.raw_name == resolved:
            return self._RANK_EXACT
        if attribute_matches(definition.raw_name, resolved):
            return self._RANK_SUBSTITUTION
        return None

    def next(self, resolved_name: str, raw_name: str = None) -> NodeDefinition | None:
        """
        Return the most specific matching definition and advance the cursor.

        Definitions are ranked, so an unspecific definition like
        'name="$(var container_name)"' or 'name=container_name' cannot swallow
        the following literal definitions of the same file.
        """
        resolved = os.path.basename(resolved_name or "")
        best_index: int | None = None
        best_rank = self._RANK_WILDCARD + 1

        for index in range(self._index, len(self._definitions)):
            definition = self._definitions[index]
            served = self._served.get(index)

            if served is not None and resolved in served:
                # already produced this name, look further down the file
                continue

            rank = self._match_rank(definition, resolved, raw_name)

            if rank is None or rank >= best_rank:
                continue

            best_index = index
            best_rank = rank

            if rank == self._RANK_EXACT:
                # nothing can be better, stop scanning
                break

        if best_index is not None:
            return self._take(best_index, resolved)

        Log.debug(f"NodeDefinitionCursor: no definition found for '{resolved}'")
        return None

    def file_range(self, resolved_name: str, raw_name: str = None) -> dict[str, int] | None:
        """
        Return a mutable copy of the next matching file range.

        The copy protects the shared cache entry against modification by
        callers.
        """
        definition = self.next(resolved_name, raw_name)
        return dict(definition.file_range) if definition is not None else None
