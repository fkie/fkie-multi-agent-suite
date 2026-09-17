# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import copy
import os
import re
import threading
import time
from collections import OrderedDict
from collections.abc import Callable, Hashable
from functools import lru_cache
from typing import Any, Optional

from fkie_mas_pylib.logging.logging import Log


class _Missing:
    """Sentinel used to distinguish missing values from cached None."""

    def __repr__(self) -> str:
        return "<MISSING>"


MISSING = _Missing()

FileFingerprint = Optional[tuple[int, int, int, int]]


def normalize_path(path: str) -> str:
    """
    Return an absolute, normalized path without resolving symlinks.

    This function is used for stable logical path handling. Callers that need
    the physical target of a symlink must additionally use os.path.realpath().
    """
    return os.path.abspath(os.path.normpath(os.fspath(path)))


def stat_fingerprint(path: str) -> FileFingerprint:
    """
    Return a file fingerprint based on device, inode, modification time and size.

    os.stat() follows symlinks. A return value of None means that the file could
    not be stat'ed.
    """
    try:
        stat_result = os.stat(normalize_path(path))
    except OSError:
        return None

    return (
        stat_result.st_dev,
        stat_result.st_ino,
        stat_result.st_mtime_ns,
        stat_result.st_size,
    )


class LruTtlCache:
    """
    Thread-safe cache with LRU bound, optional TTL and fingerprint validation.

    The cache stores arbitrary hashable keys and arbitrary values. The
    fingerprint is optional and can be used to detect changes of an external
    source without explicitly invalidating the cache entry.

    TTL semantics:
      * ttl == 0 disables expiration
      * ttl > 0 defines a fixed lifetime starting at insertion time
      * accesses do not extend the lifetime

    Threading contract:
      * the internal lock protects only cache state
      * value factories run outside the cache lock
      * concurrent duplicate factory calls are allowed
      * the last completed writer wins
      * no external lock must be acquired while holding this lock
    """

    def __init__(self, maxsize: int = 128, ttl: float = 0.0, name: str = "") -> None:
        """
        :param maxsize:
            Maximum number of entries. The least recently used entries are
            removed first.
        :param ttl:
            Entry lifetime in seconds. Zero disables expiration.
        :param name:
            Name used in statistics and diagnostics.
        """
        if maxsize < 1:
            raise ValueError("maxsize must be greater than zero")
        if ttl < 0.0:
            raise ValueError("ttl must not be negative")

        self._maxsize = maxsize
        self._ttl = ttl
        self._name = name or self.__class__.__name__

        self._lock = threading.Lock()

        # key -> (value, fingerprint, insertion timestamp)
        self._entries: OrderedDict[Hashable, tuple[Any, Any, float]] = OrderedDict()

        self._hits = 0
        self._misses = 0
        self._evictions = 0
        self._invalidations = 0
        self._expirations = 0

    def _purge_expired_locked(self, now: float) -> None:
        """
        Remove expired entries.

        The lock must be held. Expired entries are counted separately and not
        as evictions because they were removed due to TTL expiration and not
        because of the LRU bound.
        """
        if self._ttl <= 0.0:
            return

        expired_keys = [key for key, (_value, _fingerprint, stamp) in self._entries.items() if now - stamp >= self._ttl]

        for key in expired_keys:
            if self._entries.pop(key, None) is not None:
                self._expirations += 1

    def get(self, key: Hashable, fingerprint: Any = None) -> Any:
        """
        Return the cached value or MISSING.

        If fingerprint is not None, the cached fingerprint must match it.
        """
        now = time.monotonic()

        with self._lock:
            entry = self._entries.get(key)

            if entry is None:
                self._misses += 1
                return MISSING

            value, entry_fingerprint, stamp = entry

            if self._ttl > 0.0 and now - stamp >= self._ttl:
                # The entry outlived its TTL and is removed on access.
                self._entries.pop(key, None)
                self._expirations += 1
                self._misses += 1
                return MISSING

            if fingerprint is not None and entry_fingerprint != fingerprint:
                # The external source changed, so the cached value is stale.
                self._entries.pop(key, None)
                self._invalidations += 1
                self._misses += 1
                return MISSING

            self._entries.move_to_end(key)
            self._hits += 1
            return value

    def put(self, key: Hashable, value: Any, fingerprint: Any = None) -> None:
        """
        Store a value and enforce the maximum cache size.
        """
        now = time.monotonic()

        with self._lock:
            self._purge_expired_locked(now)

            # Assigning an existing key keeps its old position, therefore the
            # entry is explicitly moved to the most recently used end.
            self._entries[key] = (value, fingerprint, now)
            self._entries.move_to_end(key)

            while len(self._entries) > self._maxsize:
                self._entries.popitem(last=False)
                self._evictions += 1

    def get_or_create(self, key: Hashable, factory: Callable[[], Any], fingerprint: Any = None) -> Any:
        """
        Return a cached value or create it using factory.

        The factory always runs outside the cache lock. Concurrent duplicate
        factory calls are therefore possible and the last completed writer
        wins. This is intentional because factories may call back into code
        that uses other locks and single-flight behaviour could deadlock.
        """
        value = self.get(key, fingerprint)

        if value is not MISSING:
            return value

        value = factory()

        if value is MISSING:
            # MISSING is reserved as "no value" marker and must never be
            # stored, otherwise it could not be distinguished from a miss.
            Log.debug(f"{self._name}: factory for {key!r} returned MISSING, result is not cached")
            return value

        self.put(key, value, fingerprint)
        return value

    def invalidate(self, key: Hashable) -> bool:
        """
        Remove one entry.

        :return:
            True if an entry was removed.
        """
        with self._lock:
            if key not in self._entries:
                return False

            del self._entries[key]
            self._invalidations += 1
            return True

    def invalidate_if(self, predicate: Callable[[Hashable], bool]) -> int:
        """
        Remove all entries whose keys match predicate.

        The predicate is evaluated while the cache lock is held and must
        therefore not acquire another lock or perform blocking operations.
        """
        with self._lock:
            keys = [key for key in self._entries if predicate(key)]

            for key in keys:
                del self._entries[key]

            self._invalidations += len(keys)
            return len(keys)

    def clear(self) -> int:
        """
        Remove all entries.

        Hit and miss counters are intentionally preserved so that the overall
        efficiency of the cache stays observable. Removed entries are counted
        as invalidations, consistent with invalidate() and invalidate_if().

        :return:
            Number of removed entries.
        """
        with self._lock:
            removed = len(self._entries)
            self._entries.clear()
            self._invalidations += removed
            return removed

    def statistics(self) -> dict[str, Any]:
        """
        Return cache statistics.

        Expired entries are purged first so that the reported size matches the
        number of entries that could actually be served.
        """
        with self._lock:
            self._purge_expired_locked(time.monotonic())

            total = self._hits + self._misses

            return {
                "name": self._name,
                "size": len(self._entries),
                "maxsize": self._maxsize,
                "ttl": self._ttl,
                "hits": self._hits,
                "misses": self._misses,
                "evictions": self._evictions,
                "expirations": self._expirations,
                "invalidations": self._invalidations,
                "hit_rate": self._hits / total if total else 0.0,
            }


class FileContentCache:
    """
    Cache for text content of small files.

    Entries are validated with a file fingerprint. Therefore external file
    changes are detected even when no watchdog event was received.

    The cache key contains the physical path and encoding. Different symlink
    aliases to the same file share the same cached content.
    """

    def __init__(
        self,
        maxsize: int = 256,
        ttl: float = 0.0,
        max_file_size: int = 5 * 1024 * 1024,
    ) -> None:
        """
        :param maxsize:
            Maximum number of cached files.
        :param ttl:
            Entry lifetime in seconds. Zero disables expiration, which is safe
            because every entry is validated with a file fingerprint.
        :param max_file_size:
            Files larger than this limit are read but never cached.
        """
        if max_file_size < 0:
            raise ValueError("max_file_size must not be negative")

        self._max_file_size = max_file_size
        self._cache = LruTtlCache(maxsize=maxsize, ttl=ttl, name="FileContentCache")

    @staticmethod
    def fingerprint(path: str) -> FileFingerprint:
        """
        Return the current fingerprint of a file.
        """
        return stat_fingerprint(path)

    @staticmethod
    def _cache_path(path: str) -> str:
        """
        Return the physical cache path.

        realpath() allows different symlink aliases to share one cache entry.
        """
        return os.path.realpath(normalize_path(path))

    def get_content(self, path: str, encoding: str = "utf-8", default: Any = MISSING) -> Any:
        """
        Return the text content of a file.

        Files that do not exist, are too large or cannot be decoded are not
        cached. In these cases default is returned.
        """
        normalized_path = normalize_path(path)
        cache_path = self._cache_path(normalized_path)
        fingerprint = self.fingerprint(normalized_path)

        if fingerprint is None:
            return default

        file_size = fingerprint[3]

        if file_size > self._max_file_size:
            Log.debug(
                f"FileContentCache: skip caching of {normalized_path}, "
                f"size {file_size} exceeds limit {self._max_file_size}"
            )
            return self._read(normalized_path, encoding, default)

        cache_key = (cache_path, encoding)
        cached = self._cache.get(cache_key, fingerprint)

        if cached is not MISSING:
            return cached

        content = self._read(normalized_path, encoding, MISSING)

        if content is MISSING:
            return default

        # Avoid caching content that was read while the file was changing.
        if self.fingerprint(normalized_path) == fingerprint:
            self._cache.put(cache_key, content, fingerprint)
        else:
            Log.debug(f"FileContentCache: {normalized_path} changed while reading, result is not cached")

        return content

    @staticmethod
    def _read(path: str, encoding: str, default: Any) -> Any:
        """
        Read a text file using strict decoding.
        """
        try:
            with open(path, encoding=encoding) as file_object:
                return file_object.read()
        except (OSError, UnicodeError, LookupError) as error:
            Log.debug(f"FileContentCache: cannot read {path}: {error}")
            return default

    def invalidate(self, path: str) -> int:
        """
        Invalidate all cached encodings of one physical file.

        :return:
            Number of removed entries.
        """
        cache_path = self._cache_path(path)

        return self._cache.invalidate_if(lambda key: isinstance(key, tuple) and len(key) >= 1 and key[0] == cache_path)

    def clear(self) -> None:
        """
        Remove all cached file contents.
        """
        self._cache.clear()

    def statistics(self) -> dict[str, Any]:
        """
        Return cache statistics.
        """
        return self._cache.statistics()


class MessageStructCache:
    """
    Cache for expanded message, service and action field structures.

    Interface definitions are immutable during the process lifetime, so no TTL
    is required. A deep copy is returned because callers may add
    request-specific values to the returned structure.

    Note on cost: the deep copy is intentional and required for correctness.
    The cache therefore only saves the introspection of the interface
    definition, not the copy itself.
    """

    def __init__(self, maxsize: int = 512) -> None:
        """
        :param maxsize:
            Maximum number of cached interface structures.
        """
        self._cache = LruTtlCache(maxsize=maxsize, name="MessageStructCache")

    def get_or_create(
        self,
        kind: str,
        type_name: str,
        factory: Callable[[], list[dict[str, Any]]],
    ) -> list[dict[str, Any]]:
        """
        Return a mutable copy of the cached message structure.
        """
        key = (kind, type_name)
        value = self._cache.get_or_create(key, factory)

        # Never expose the cached mutable object to callers.
        return copy.deepcopy(value)

    def clear(self) -> None:
        """
        Remove all cached message structures.
        """
        self._cache.clear()

    def statistics(self) -> dict[str, Any]:
        """
        Return cache statistics.
        """
        return self._cache.statistics()


@lru_cache(maxsize=512)
def compile_regex(pattern: str, flags: int = 0) -> "re.Pattern":
    """
    Compile and cache a runtime-created regular expression.

    re.compile() already maintains an internal cache, but that cache is
    cleared whenever it overflows and is not observable. This wrapper provides
    a stable, measurable cache for patterns that are created at runtime.

    Statically known regular expressions should be compiled once at module
    level instead.
    """
    return re.compile(pattern, flags)


# Process-wide cache instances.
FILE_CONTENT_CACHE = FileContentCache()
MESSAGE_STRUCT_CACHE = MessageStructCache()


def cache_statistics() -> dict[str, Any]:
    """
    Return statistics for all process-wide caches.
    """
    # Imported lazily to avoid a circular import at module load time.
    from .launch_argument_cache import LAUNCH_ARGUMENT_CACHE
    from .launch_definition_index import NODE_DEFINITION_CACHE

    regex_info = compile_regex.cache_info()

    return {
        "launch_arguments": LAUNCH_ARGUMENT_CACHE.statistics(),
        "file_content": FILE_CONTENT_CACHE.statistics(),
        "message_struct": MESSAGE_STRUCT_CACHE.statistics(),
        "node_definitions": NODE_DEFINITION_CACHE.statistics(),
        "regex": {
            "hits": regex_info.hits,
            "misses": regex_info.misses,
            "size": regex_info.currsize,
            "maxsize": regex_info.maxsize,
        },
    }
