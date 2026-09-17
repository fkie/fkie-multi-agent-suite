# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import re
from collections.abc import Callable
from typing import Any, NamedTuple

from fkie_mas_pylib.logging.logging import Log

from .caches import MISSING, LruTtlCache, normalize_path, stat_fingerprint


class _Uncacheable:
    """
    Sentinel marking a cache key that must never be used for caching.

    A key becomes uncacheable when the launch file cannot be stat'ed or when
    the launch context cannot be described in a stable, reproducible way. In
    both cases a cache hit could return results that belong to a different
    file state or a different context.
    """

    def __repr__(self) -> str:
        return "<UNCACHEABLE>"


UNCACHEABLE = _Uncacheable()


# Default repr of objects without __repr__ contains the memory address and is
# therefore not stable between equal objects.
_OBJECT_ADDRESS_PATTERN = re.compile(r" at 0x[0-9a-fA-F]+")


class LaunchArgTemplate(NamedTuple):
    """
    Immutable, request-independent description of a top-level launch argument.
    """

    name: str
    default_value: str | None
    description: str | None
    choices: tuple[str, ...]


class LaunchArgumentCache:
    """
    Shared TTL/LRU cache for parsed top-level launch arguments.

    Cached values contain only request-independent LaunchArgTemplate objects.
    The concrete argument value is always derived from the current request.
    Therefore values cannot leak between callers.

    The cache key consists of:

    * the physical launch file path
    * the file fingerprint
    * the launch context fingerprint

    Because the file fingerprint is part of the key, a modification of the
    top-level launch file always produces a new key and cannot deliver a stale
    result. Included launch files are not covered by that fingerprint and rely
    on invalidation through the file observer. A positive TTL can be used as
    an additional safety net if included files are not observed reliably.
    """

    def __init__(self, ttl: float = 0.0, max_size: int = 64) -> None:
        """
        :param ttl:
            Entry lifetime in seconds. Zero disables expiration.
        :param max_size:
            Maximum number of cached launch files and context variants.
        """
        self._cache = LruTtlCache(
            maxsize=max_size,
            ttl=ttl,
            name="LaunchArgumentCache",
        )

    @staticmethod
    def _stat_fingerprint(path: str):
        """
        Return the shared file fingerprint used by all file-based caches.
        """
        return stat_fingerprint(path)

    @classmethod
    def _stable_text(cls, value: Any) -> str | None:
        """
        Return a stable textual representation of a launch configuration value.

        Launch configuration values may be plain strings, lists of strings or
        launch Substitution objects. Substitutions without a custom __repr__
        would be rendered including their memory address, which changes on
        every parse run and would make the cache key useless.

        :return:
            Stable text or None if no stable representation is available.
        """
        if value is None or isinstance(value, (bool, int, float)):
            return repr(value)

        if isinstance(value, str):
            return value

        if isinstance(value, bytes):
            return repr(value)

        if isinstance(value, (list, tuple)):
            parts = []

            for item in value:
                text = cls._stable_text(item)

                if text is None:
                    return None

                parts.append(text)

            return "\x00".join(parts)

        # launch.Substitution implementations provide describe().
        describe = getattr(value, "describe", None)

        if callable(describe):
            try:
                return str(describe())
            except Exception:
                return None

        text = repr(value)

        if _OBJECT_ADDRESS_PATTERN.search(text):
            # The representation contains a memory address and is not stable.
            return None

        return text

    @classmethod
    def _context_fingerprint(cls, context) -> tuple[tuple[str, str], ...] | None:
        """
        Return a stable fingerprint for launch configuration values.

        Launch argument defaults may refer to LaunchConfiguration values, so
        the current launch context must be part of the cache key.

        :return:
            Sorted tuple of key/value pairs or None if the context cannot be
            described in a stable way. None must be treated as "do not cache",
            because falling back to an empty fingerprint would merge different
            contexts onto the same cache key.
        """
        try:
            configurations = context.launch_configurations.items()
        except Exception as error:
            Log.debug(f"LaunchArgumentCache: cannot read launch configurations: {error}")
            return None

        entries = []

        try:
            for key, value in configurations:
                key_text = cls._stable_text(key)
                value_text = cls._stable_text(value)

                if key_text is None or value_text is None:
                    Log.debug(
                        f"LaunchArgumentCache: launch configuration {key!r} "
                        f"has no stable representation, caching is disabled "
                        f"for this request"
                    )
                    return None

                entries.append((key_text, value_text))
        except Exception as error:
            Log.debug(f"LaunchArgumentCache: cannot build context fingerprint: {error}")
            return None

        return tuple(sorted(entries))

    def make_key(self, path: str, context) -> tuple:
        """
        Build a cache key for a launch file and launch context.

        If the launch file cannot be stat'ed or the context cannot be described
        stably, the returned key contains the UNCACHEABLE sentinel. Such keys
        are never stored and never produce a hit.
        """
        normalized_path = normalize_path(path)
        real_path = os.path.realpath(normalized_path)

        file_fingerprint = self._stat_fingerprint(real_path)
        context_fingerprint = self._context_fingerprint(context)

        if file_fingerprint is None or context_fingerprint is None:
            return (real_path, UNCACHEABLE, UNCACHEABLE)

        return (
            real_path,
            file_fingerprint,
            context_fingerprint,
        )

    @staticmethod
    def is_cacheable(key: tuple) -> bool:
        """
        Return whether a key created by make_key() may be cached.
        """
        return UNCACHEABLE not in key

    def get(self, key: tuple) -> tuple[LaunchArgTemplate, ...] | None:
        """
        Return cached templates or None on a cache miss.

        An empty tuple is a valid cached result and is not considered a miss.
        """
        if not self.is_cacheable(key):
            return None

        value = self._cache.get(key)

        if value is MISSING:
            return None

        return value

    def put(self, key: tuple, templates: tuple[LaunchArgTemplate, ...]) -> None:
        """
        Store parsed launch argument templates.

        Uncacheable keys are silently ignored.
        """
        if not self.is_cacheable(key):
            Log.debug("LaunchArgumentCache: key is not cacheable, result is not stored")
            return

        self._cache.put(key, templates)

    def get_or_load(
        self, key: tuple, loader: Callable[[], tuple[LaunchArgTemplate, ...]]
    ) -> tuple[LaunchArgTemplate, ...]:
        """
        Return cached templates or load them using loader.

        The loader runs outside the cache lock. Concurrent duplicate parsing is
        allowed and the last completed result is stored. Uncacheable keys always
        invoke the loader.
        """
        if not self.is_cacheable(key):
            return loader()

        return self._cache.get_or_create(key, loader)

    def invalidate(self, path: str) -> int:
        """
        Invalidate all cache entries belonging to one launch file.

        All context variants and all older file fingerprints are removed.
        """
        normalized_path = normalize_path(path)
        real_path = os.path.realpath(normalized_path)

        return self._cache.invalidate_if(lambda key: isinstance(key, tuple) and len(key) >= 1 and key[0] == real_path)

    def clear(self) -> None:
        """
        Remove all cached launch argument templates.
        """
        self._cache.clear()

    def statistics(self) -> dict[str, object]:
        """
        Return cache statistics using the common cache schema.
        """
        return self._cache.statistics()


# The cache must be shared between all LaunchConfig instances and requests.
# No TTL is used because the file fingerprint of the top-level launch file is
# part of the cache key and included files are invalidated by the observer.
LAUNCH_ARGUMENT_CACHE = LaunchArgumentCache(
    ttl=0.0,
    max_size=64,
)


def invalidate_launch_argument_cache(path: str | None = None) -> None:
    """
    Invalidate launch argument cache entries.

    If path is None, the complete cache is cleared.
    """
    if path is None:
        LAUNCH_ARGUMENT_CACHE.clear()
    else:
        LAUNCH_ARGUMENT_CACHE.invalidate(path)
