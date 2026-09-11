# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import errno
import resource
import os
import queue
import threading
import time
import traceback
from typing import Any
from typing import Callable
from typing import Dict
from typing import FrozenSet
from typing import Iterable
from typing import List
from typing import Optional
from typing import Set
from typing import Tuple

from watchdog.events import FileSystemEvent
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer

from fkie_mas_pylib.logging.logging import Log

_INOTIFY_SYSFS = "/proc/sys/fs/inotify"


def _read_inotify_limit(name: str) -> int:
    """Read an inotify limit from procfs, -1 if not available."""
    try:
        with open(os.path.join(_INOTIFY_SYSFS, name)) as handle:
            return int(handle.read().strip())
    except Exception:
        return -1


def _count_inotify_instances(pid: str = "self") -> int:
    """Count open inotify instances (file descriptors) of a process."""
    count = 0
    fd_dir = f"/proc/{pid}/fd"
    try:
        names = os.listdir(fd_dir)
    except OSError:
        return -1
    for name in names:
        try:
            if os.readlink(os.path.join(fd_dir, name)) == "anon_inode:inotify":
                count += 1
        except OSError:
            continue
    return count


def _count_inotify_instances_of_user() -> int:
    """Count inotify instances of all processes owned by the current user."""
    uid = os.getuid()
    total = 0
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            if os.stat(f"/proc/{entry}").st_uid != uid:
                continue
        except OSError:
            continue
        count = _count_inotify_instances(entry)
        if count > 0:
            total += count
    return total


def _open_fd_count(pid: str = "self") -> int:
    """Count open file descriptors of a process."""
    try:
        return len(os.listdir(f"/proc/{pid}/fd"))
    except OSError:
        return -1


class _ControlMessage:
    """Internal marker object used in the callback queue."""

    def __init__(self, name: str) -> None:
        self._name = name

    def __repr__(self) -> str:
        return f"<{self._name}>"


# Terminates the callback worker thread.
_SHUTDOWN = _ControlMessage("SHUTDOWN")

# Wakes up the callback worker thread to check for delayed events.
_WAKEUP = _ControlMessage("WAKEUP")

# ----------------------------------------------------------------------------
# watch root detection
# ----------------------------------------------------------------------------

# Environment variables containing colon separated prefix paths.
_PREFIX_PATH_VARS: Tuple[str, ...] = (
    "COLCON_PREFIX_PATH",
    "AMENT_PREFIX_PATH",
)

# Marker files identifying a colcon/ament install prefix.
_PREFIX_MARKERS: Tuple[str, ...] = (
    "local_setup.bash",
    "setup.bash",
    ".colcon_install_layout",
)


def _collapse_prefix(path: str) -> str:
    """
    Collapse a per package prefix to its workspace install prefix.

    AMENT_PREFIX_PATH lists <ws>/install/<pkg> for every package. Watching the
    common parent <ws>/install instead keeps the number of watch roots small.
    """
    parent = os.path.dirname(path)

    if not parent or parent == path:
        return path

    for marker in _PREFIX_MARKERS:
        if os.path.exists(os.path.join(parent, marker)):
            return parent

    return path


def ros_distro_lib_root() -> Optional[str]:
    """
    Return <ros_prefix>/lib of the system installation.

    Node executables of system packages live here. Adding it as a recursive
    watch root replaces one inotify instance per package directory with a
    single instance for the whole tree.
    """
    for entry in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not entry:
            continue

        prefix = FileObserverRegistry._normalize_path(entry)

        if not prefix.startswith("/opt/ros" + os.sep):
            continue

        # /opt/ros/<distro>/... -> /opt/ros/<distro>
        parts = prefix.split(os.sep)
        distro_prefix = os.sep.join(parts[:4])
        lib = os.path.join(distro_prefix, "lib")

        if os.path.isdir(lib):
            return lib

    return None


def default_watch_roots(
    include_sources: bool = True,
    include_build: bool = True,
    include_ros: bool = True,
    exclude_prefixes: Iterable[str] = (),
    env_vars: Iterable[str] = _PREFIX_PATH_VARS
) -> Tuple[str, ...]:
    """
    Derive recursive watch roots from the ROS/colcon environment.

    Both the logical path and the resolved real path of every prefix are added,
    because the registry tracks observed directories with both variants.

    :param include_sources:
        Also add the <ws>/src directory of a workspace, which is the symlink
        target of share files in a --symlink-install workspace.
    :param include_build:
        Also add the <ws>/build directory of a workspace. Node executables in
        install/<pkg>/lib are symlinks into build/<pkg> for a --symlink-install
        workspace, so without this root every started node would need its own
        inotify instance.
    :param exclude_prefixes:
        Paths that are skipped, for example the read only system installation.
        Files below these paths are watched individually as before.
    :param env_vars:
        Environment variables holding os.pathsep separated prefix paths.
    :return:
        Existing directories, nested entries removed, shortest path first.
    """
    excluded = tuple(
        FileObserverRegistry._normalize_path(path)
        for path in exclude_prefixes or ())

    candidates: List[str] = []

    def _append(path: str) -> None:
        """Add a path and its real path once, keeping the insertion order."""
        for entry in (path, os.path.realpath(path)):
            if entry and entry not in candidates:
                candidates.append(entry)

    for variable in env_vars:
        for entry in os.environ.get(variable, "").split(os.pathsep):
            if not entry:
                continue

            prefix = _collapse_prefix(
                FileObserverRegistry._normalize_path(entry))

            if not os.path.isdir(prefix):
                continue

            _append(prefix)

            # Directories located next to the install prefix of a workspace.
            sibling_dirs: List[str] = []

            if include_sources:
                sibling_dirs.append("src")

            if include_build:
                sibling_dirs.append("build")

            workspace = os.path.dirname(prefix)

            for name in sibling_dirs:
                sibling = os.path.join(workspace, name)

                if os.path.isdir(sibling):
                    _append(sibling)

    _append(ros_distro_lib_root())

    roots: List[str] = []

    # Shortest first, so a parent is kept and all its children are dropped.
    for candidate in sorted(candidates, key=len):
        if any(
            candidate == skip or candidate.startswith(skip + os.sep)
            for skip in excluded
        ):
            continue

        if any(
            candidate == root or candidate.startswith(root + os.sep)
            for root in roots
        ):
            continue

        roots.append(candidate)

    return tuple(roots)


class _DirectoryWatch:
    """
    Reference counted watchdog watch for one watch target.

    A watch target is either a single observed directory (non recursive) or a
    configured watch root covering a whole directory tree (recursive).

    The watch object is None while the corresponding watchdog schedule() call
    is still pending. The generation is used to detect that an entry was
    removed and recreated while a schedule() call was in progress.
    """

    __slots__ = ("watch", "count", "generation", "recursive")

    def __init__(
        self,
        watch: Any,
        count: int,
        generation: int,
        recursive: bool = False
    ) -> None:
        self.watch = watch
        self.count = count
        self.generation = generation
        self.recursive = recursive

    def __repr__(self) -> str:
        return (
            f"<_DirectoryWatch count={self.count} "
            f"generation={self.generation} "
            f"recursive={self.recursive} "
            f"scheduled={self.watch is not None}>")


class FileObserverHandler(FileSystemEventHandler):
    """
    Watchdog event handler forwarding relevant file system events.

    Filtering of directories, path mapping, debouncing and dispatching to
    affected launch files are handled by FileObserverRegistry.

    Directory events are only forwarded for deletion and move, because those
    events indicate that observed files disappeared or that a watched directory
    is gone. Directory modification events are ignored, since every change of
    a contained file also triggers a modification of its directory.
    """

    IGNORED_EVENT_TYPES = frozenset({
        'opened',
        'closed',
        'closed_no_write',
    })

    FORWARDED_DIRECTORY_EVENT_TYPES = frozenset({
        'deleted',
        'moved',
    })

    def __init__(
        self,
        callback: Callable[[str, str, Optional[str], bool], None]
    ) -> None:
        """
        :param callback:
            Called with event type, source path, optional destination path and
            a flag indicating whether the event refers to a directory.
        """
        super().__init__()
        self._callback = callback

    def _forward(
        self,
        event_type: str,
        src_path: str,
        dest_path: Optional[str] = None,
        is_directory: bool = False
    ) -> None:
        """
        Forward one event to the registry.

        Exceptions are logged and never propagated into the watchdog thread,
        because an exception there would terminate the whole event emitter.
        """
        if event_type in self.IGNORED_EVENT_TYPES:
            return

        if (
            is_directory
            and event_type not in self.FORWARDED_DIRECTORY_EVENT_TYPES
        ):
            return

        try:
            self._callback(event_type, src_path, dest_path, is_directory)
        except Exception:
            Log.warn(
                f"{self.__class__.__name__}: error while handling "
                f"{event_type} event:\n{traceback.format_exc()}")

    def on_modified(self, event: FileSystemEvent) -> None:
        self._forward(
            event.event_type,
            event.src_path,
            None,
            event.is_directory)

    def on_created(self, event: FileSystemEvent) -> None:
        self._forward(
            event.event_type,
            event.src_path,
            None,
            event.is_directory)

    def on_deleted(self, event: FileSystemEvent) -> None:
        self._forward(
            event.event_type,
            event.src_path,
            None,
            event.is_directory)

    def on_moved(self, event: FileSystemEvent) -> None:
        dest_path = getattr(event, 'dest_path', None)

        self._forward(
            event.event_type,
            event.src_path,
            dest_path,
            event.is_directory)


class FileObserverRegistry:
    """
    Thread-safe registry for observed files.

    The registry owns one watchdog Observer and keeps reference counts for
    observed files and watch targets. The same file can therefore be requested
    by several launch files or nodes without losing the watch when only one
    user releases it.

    Both the logical path and the real path of a file are tracked. This is
    important for symlinks:

    * changes to the symlink target are detected in the target directory
    * replacement of the symlink itself is detected in the logical directory
    * several symlinks pointing to the same target do not overwrite each other

    Watch targets and inotify instances
    -----------------------------------
    watchdog creates one event emitter, and on Linux one inotify instance, per
    scheduled path. Observing many single directories therefore consumes many
    inotify instances and can exceed
    /proc/sys/fs/inotify/max_user_instances (errno EMFILE).

    To avoid this, directories can be grouped into recursive watch roots. Every
    observed directory below such a root is covered by one single recursive
    watch of that root. The individual subdirectories then only count against
    max_user_watches, which is orders of magnitude larger. Events of unrelated
    files inside a root are discarded by the normal path resolution, so the
    externally visible behaviour is unchanged.

    Without watch_roots the registry behaves exactly like before: one non
    recursive watch per observed directory.

    Supported events are:

    * modified
    * created
    * deleted
    * moved

    The public change callback receives:

        (event_type, observed_path, affected_launch_files)

    For a move event, the callback may be called once for the source path and
    once for the destination path if both paths are relevant.

    Debouncing uses a leading and a trailing edge. The first event for a path
    is forwarded immediately. Further events inside the interval are collapsed
    into one additional event that is forwarded when the interval has elapsed.
    Therefore the last change of a file is never lost.

    Threading contract:

    * all registry state is protected by one internal lock
    * watchdog schedule(), unschedule() and unschedule_all() are never called
      while holding the internal lock, because the watchdog dispatch thread
      calls into this registry while holding the observer lock and the reverse
      order would deadlock
    * join() is never called while holding the lock
    * the change callback runs in a dedicated worker thread and never in the
      watchdog thread, so callbacks may block, for example while publishing
      over a websocket
    * start() must not be called while stop() is still running
    """

    def __init__(
        self,
        on_change: Callable[[str, str, FrozenSet[str]], None],
        min_event_interval: float = 1.0,
        watch_roots: Optional[Iterable[str]] = None
    ) -> None:
        """
        :param on_change:
            Callback receiving event type, observed path and affected launch
            files.
        :param min_event_interval:
            Minimum interval between forwarded events for the same observed
            path. The first event is forwarded immediately, subsequent events
            during the interval are collapsed and forwarded once at the end of
            the interval.
        :param watch_roots:
            Directory trees that are observed with one single recursive watch
            each. Every observed directory inside such a tree shares this
            watch, which keeps the number of used inotify instances constant.
            For every given root its resolved real path is registered as an
            additional root, so symlinked overlays such as
            <ws>/install -> <ws>/src are covered as well.
        """
        if min_event_interval < 0.0:
            raise ValueError(
                "min_event_interval must not be negative")

        self._on_change = on_change
        self._min_event_interval = min_event_interval

        self._lock = threading.Lock()

        self._observer = Observer()
        self._handler = FileObserverHandler(self._dispatch)

        self._running = False
        self._observer_started = False
        self._observer_stopped = False
        self._stopping = False

        # Recursive watch roots, longest path first so that the most specific
        # root wins when roots are nested.
        self._watch_roots: Tuple[str, ...] = self._prepare_watch_roots(
            watch_roots)

        if self._watch_roots:
            Log.debug(
                f"{self.__class__.__name__}: recursive watch roots: "
                f"{', '.join(self._watch_roots)}")

        # Queue and worker thread decoupling the watchdog thread from user
        # callbacks.
        self._queue: "queue.Queue[Any]" = queue.Queue()
        self._worker: Optional[threading.Thread] = None

        # Monotonically increasing generation for directory watch entries.
        self._watch_generation = 0

        # Logical observed path -> reference count.
        self._file_refs: Dict[str, int] = {}

        # Logical observed path -> real path and observed directories.
        self._path_info: Dict[
            str,
            Tuple[str, Tuple[str, ...]]
        ] = {}

        # Real path -> all logical paths referring to that real path.
        self._real_paths: Dict[str, Set[str]] = {}

        # Observed directory -> reference count. Only used for diagnostics,
        # several directories can share one watch target.
        self._dir_refs: Dict[str, int] = {}

        # Watch target (directory or recursive root) -> reference counted
        # watch.
        self._dir_watches: Dict[str, _DirectoryWatch] = {}

        # Launch file -> logical observed paths belonging to it.
        self._launch_files: Dict[str, Tuple[str, ...]] = {}

        # Logical observed path -> launch files referencing it. This reverse
        # index avoids scanning all launch files for every event.
        self._launch_by_path: Dict[str, Set[str]] = {}

        # Logical observed path -> timestamp of the last forwarded event.
        self._last_event: Dict[str, float] = {}

        # Logical observed path -> (event type, timestamp when the collapsed
        # event has to be forwarded).
        self._pending_events: Dict[str, Tuple[str, float]] = {}

    # -- path helpers -----------------------------------------------------

    def log_watch_targets(self) -> None:
        """
        Log every scheduled watch target. One target equals one inotify
        instance, so this is the authoritative view on the instance usage.
        """
        with self._lock:
            entries = {
                target: (entry.recursive, entry.count, entry.watch is not None)
                for target, entry in self._dir_watches.items()
            }

        Log.info(f"{self.__class__.__name__}: {len(entries)} watch targets, "
                 f"process inotify instances={_count_inotify_instances()}")

        for target, (recursive, count, scheduled) in sorted(entries.items()):
            Log.info(f"  {target} recursive={recursive} refs={count} "
                     f"scheduled={scheduled}")

    @staticmethod
    def _normalize_path(path: str) -> str:
        """
        Return a stable logical path without resolving symlinks.

        Keeping symlinks unresolved is intentional. The logical path is needed
        to detect replacement of the symlink itself.
        """
        return os.path.abspath(os.path.normpath(os.fspath(path)))

    @classmethod
    def _prepare_watch_roots(
        cls,
        watch_roots: Optional[Iterable[str]]
    ) -> Tuple[str, ...]:
        """
        Normalize the configured watch roots.

        For every root its real path is added as an additional root, because
        observed directories are tracked with their logical and their real
        path. Nested roots are collapsed to their parent, since one recursive
        watch already covers the whole subtree. Roots are returned longest
        first, so the most specific root is selected during lookup.
        """
        candidates: List[str] = []

        for root in watch_roots or ():
            normalized = cls._normalize_path(root)

            for candidate in (normalized, os.path.realpath(normalized)):
                if candidate and candidate not in candidates:
                    candidates.append(candidate)

        collapsed: List[str] = []

        for candidate in sorted(candidates, key=len):
            if any(
                candidate == root or candidate.startswith(root + os.sep)
                for root in collapsed
            ):
                continue

            collapsed.append(candidate)

        return tuple(sorted(collapsed, key=len, reverse=True))

    def _watch_target(self, directory: str) -> Tuple[str, bool]:
        """
        Return the path to schedule for a directory and its recursive flag.

        Directories inside a configured watch root share one recursive watch of
        that root. All other directories are watched individually and non
        recursively.
        """
        for root in self._watch_roots:
            if directory == root or directory.startswith(root + os.sep):
                return root, True

        return directory, False

    @classmethod
    def _path_details(
        cls,
        path: str
    ) -> Tuple[str, Tuple[str, ...]]:
        """
        Return the real path and all directories that need to be observed.

        Usually both directories are identical. For symlinks they can differ:

        * the real target directory detects target changes
        * the logical directory detects symlink replacement
        """
        logical_path = cls._normalize_path(path)
        real_path = os.path.realpath(logical_path)

        directories = []

        for candidate in (
            os.path.dirname(real_path),
            os.path.dirname(logical_path),
        ):
            if candidate not in directories and os.path.isdir(candidate):
                directories.append(candidate)

        return real_path, tuple(directories)

    # -- life cycle -------------------------------------------------------

    def start(self) -> None:
        """
        Start the watchdog observer and the callback worker.

        The registry can be started again after stop(). Files may be registered
        before or after start().
        """
        with self._lock:
            if self._running:
                return

            if self._stopping:
                raise RuntimeError(
                    "Cannot start the file observer while it is stopping")

            if self._observer_stopped:
                self._observer = Observer()
                self._observer_stopped = False

            self._running = True

            # A fresh queue avoids delivering events that were queued before
            # the previous stop().
            self._queue = queue.Queue()

            worker = threading.Thread(
                target=self._run_worker,
                name="FileObserverCallbacks",
                daemon=True)

            self._worker = worker
            observer = self._observer

        worker.start()

        try:
            observer.start()

            with self._lock:
                self._observer_started = True
        except Exception:
            Log.error(
                f"{self.__class__.__name__}: cannot start observer:\n"
                f"{traceback.format_exc()}")

            with self._lock:
                self._running = False
                self._observer_started = False
                self._observer_stopped = True
                self._worker = None
                self._queue.put(_SHUTDOWN)

            worker.join(timeout=1.0)
            raise

    def stop(self, timeout: float = 3.0) -> None:
        """
        Stop the observer and drop all registry state.

        A subsequent start() creates a new watchdog Observer. Callbacks that
        are already executing when stop() is called may finish normally.
        """
        with self._lock:
            if self._stopping:
                return

            self._stopping = True
            self._running = False

            self._file_refs.clear()
            self._path_info.clear()
            self._real_paths.clear()
            self._dir_refs.clear()
            self._dir_watches.clear()
            self._launch_files.clear()
            self._launch_by_path.clear()
            self._last_event.clear()
            self._pending_events.clear()

            observer = self._observer
            observer_started = self._observer_started
            worker = self._worker

            self._worker = None

            if worker is not None:
                self._queue.put(_SHUTDOWN)

        try:
            # All watchdog calls run without the internal lock.
            try:
                observer.unschedule_all()
            except Exception:
                Log.debug(
                    f"{self.__class__.__name__}: unscheduling observer failed:\n"
                    f"{traceback.format_exc()}")

            try:
                observer.stop()
            except Exception:
                Log.debug(
                    f"{self.__class__.__name__}: stopping observer failed:\n"
                    f"{traceback.format_exc()}")

            if observer_started:
                try:
                    observer.join(timeout=timeout)
                except Exception:
                    Log.debug(
                        f"{self.__class__.__name__}: joining observer failed:\n"
                        f"{traceback.format_exc()}")

            if worker is not None:
                try:
                    worker.join(timeout=timeout)
                except Exception:
                    Log.debug(
                        f"{self.__class__.__name__}: joining callback worker "
                        f"failed:\n{traceback.format_exc()}")

                if worker.is_alive():
                    Log.debug(
                        f"{self.__class__.__name__}: callback worker did not "
                        f"terminate within {timeout} seconds")
        finally:
            with self._lock:
                self._observer_started = False
                self._observer_stopped = True
                self._stopping = False

    def _prepare_observer_locked(self) -> None:
        """
        Prepare a new Observer after stop().

        The lock must be held.
        """
        if self._stopping:
            raise RuntimeError(
                "Cannot modify the file observer while it is stopping")

        if self._observer_stopped:
            self._observer = Observer()
            self._observer_stopped = False

    # -- watchdog operations ----------------------------------------------

    def _reserve_directory_locked(
        self,
        directory: str,
        count: int,
        pending_schedule: List[Tuple[Any, str, int, bool]]
    ) -> None:
        """
        Add references for one directory and record a required schedule call.

        The directory is mapped to its watch target first. Several directories
        inside the same recursive watch root therefore share one single watch.

        The lock must be held. The watchdog schedule() call itself is executed
        later by _flush_watch_operations() without the lock.
        """
        self._dir_refs[directory] = self._dir_refs.get(directory, 0) + count

        target, recursive = self._watch_target(directory)
        entry = self._dir_watches.get(target)

        if entry is not None:
            entry.count += count
            return

        self._watch_generation += 1

        if recursive:
            Log.debug(
                f"{self.__class__.__name__}: observe directory {directory} "
                f"through recursive watch root: {target}")
        else:
            Log.debug(
                f"{self.__class__.__name__}: observe directory: {directory}")

        self._dir_watches[target] = _DirectoryWatch(
            None,
            count,
            self._watch_generation,
            recursive)

        pending_schedule.append(
            (self._observer, target, self._watch_generation, recursive))

    def _release_directory_locked(
        self,
        directory: str,
        pending_unschedule: List[Tuple[Any, str, Any]]
    ) -> None:
        """
        Remove one directory reference and record a required unschedule call.

        The lock must be held. The watchdog unschedule() call itself is
        executed later by _flush_watch_operations() without the lock.
        """
        dir_count = self._dir_refs.get(directory, 0)

        if dir_count <= 1:
            self._dir_refs.pop(directory, None)
        else:
            self._dir_refs[directory] = dir_count - 1

        target, _recursive = self._watch_target(directory)
        entry = self._dir_watches.get(target)

        if entry is None:
            return

        entry.count -= 1

        if entry.count > 0:
            return

        del self._dir_watches[target]

        Log.debug(
            f"{self.__class__.__name__}: remove directory from observer: "
            f"{target}")

        if entry.watch is not None:
            pending_unschedule.append(
                (self._observer, target, entry.watch))
        # If the watch is still pending, _flush_watch_operations() detects the
        # missing entry through the generation check and unschedules it there.

    def _flush_watch_operations(
        self,
        pending_schedule: List[Tuple[Any, str, int, bool]],
        pending_unschedule: List[Tuple[Any, str, Any]]
    ) -> List[Tuple[str, BaseException]]:
        """
        Execute all pending watchdog operations without holding the lock.

        Unschedule operations run first so that a watch target which is
        released and immediately re-referenced ends up with a valid watch.

        :return:
            List of watch targets that could not be scheduled together with the
            raised exception.
        """
        failures: List[Tuple[str, BaseException]] = []

        for observer, target, watch in pending_unschedule:
            with self._lock:
                # The target may have been referenced again in the meantime.
                entry = self._dir_watches.get(target)
                still_referenced = (
                    entry is not None
                    and observer is self._observer)

            if still_referenced:
                Log.debug(
                    f"{self.__class__.__name__}: keep directory {target}, "
                    f"it was referenced again before unscheduling")
                continue

            try:
                observer.unschedule(watch)
            except Exception:
                Log.debug(
                    f"{self.__class__.__name__}: unschedule {target} "
                    f"failed:\n{traceback.format_exc()}")

        for observer, target, generation, recursive in pending_schedule:
            try:
                watch = observer.schedule(
                    self._handler,
                    target,
                    recursive=recursive)
            except Exception as error:
                with self._lock:
                    entry = self._dir_watches.get(target)

                    if (
                        entry is not None
                        and entry.generation == generation
                        and entry.watch is None
                    ):
                        # Drop the reservation, there is no usable watch.
                        del self._dir_watches[target]

                Log.debug(
                    f"{self.__class__.__name__}: schedule {target} "
                    f"failed:\n{traceback.format_exc()}")

                failures.append((target, error))
                continue

            obsolete_watch = None

            with self._lock:
                entry = self._dir_watches.get(target)

                if (
                    entry is not None
                    and entry.generation == generation
                    and observer is self._observer
                ):
                    entry.watch = watch
                else:
                    # The reservation is gone or belongs to another generation
                    # or observer, so this watch is not needed any more.
                    obsolete_watch = watch

            if obsolete_watch is not None:
                try:
                    observer.unschedule(obsolete_watch)
                except Exception:
                    Log.debug(
                        f"{self.__class__.__name__}: unschedule obsolete watch "
                        f"for {target} failed:\n{traceback.format_exc()}")

        return failures

    # -- single files -----------------------------------------------------

    def add_file(self, path: str) -> None:
        """
        Observe a file or symlink.

        Reference counting is applied to both the file and all required
        directories. Calling add_file() twice requires two remove_file() calls.

        :raise OSError:
            If one of the required directories cannot be observed.
        :raise RuntimeError:
            If called while the observer is stopping.
        """
        logical_path = self._normalize_path(path)

        pending_schedule: List[Tuple[Any, str, int, bool]] = []
        pending_unschedule: List[Tuple[Any, str, Any]] = []

        with self._lock:
            self._prepare_observer_locked()

            existing_count = self._file_refs.get(logical_path, 0)

            if existing_count > 0:
                path_info = self._path_info[logical_path]
                real_path, directories = path_info
            else:
                real_path, directories = self._path_details(logical_path)

                if not directories:
                    raise OSError(
                        f"no existing directory available for observation: "
                        f"{logical_path}"
                    )

                self._path_info[logical_path] = (
                    real_path,
                    directories)

                self._real_paths.setdefault(real_path, set()).add(
                    logical_path)

            self._file_refs[logical_path] = existing_count + 1
            print(f"ADD OBSERVE: {logical_path}")

            for directory in directories:
                self._reserve_directory_locked(
                    directory,
                    1,
                    pending_schedule)

        failures = self._flush_watch_operations(
            pending_schedule,
            pending_unschedule)

        if not failures:
            return

        # Collect diagnostics before the rollback releases the reservations.
        with self._lock:
            watched_dirs = len(self._dir_watches)
            observed_dirs = len(self._dir_refs)
            watch_roots = len(self._watch_roots)

        diagnostics = (
            f"registry watches={watched_dirs}, "
            f"observed directories={observed_dirs}, "
            f"recursive watch roots={watch_roots}, "
            f"inotify instances: process={_count_inotify_instances()}, "
            f"user={_count_inotify_instances_of_user()}, "
            f"limit={_read_inotify_limit('max_user_instances')}; "
            f"open fds={_open_fd_count()}, "
            f"soft fd limit={resource.getrlimit(resource.RLIMIT_NOFILE)[0]}")

        if watch_roots:
            hint = (
                "Increase the limit with: "
                "sudo sysctl -w fs.inotify.max_user_instances=1024, "
                "raise ulimit -n, or reduce the number of watch roots.")
        else:
            hint = (
                "Increase the limit with: "
                "sudo sysctl -w fs.inotify.max_user_instances=1024, "
                "raise ulimit -n, or configure recursive watch roots.")

        # Roll back the complete registration, otherwise the file would be
        # registered without a working watch.
        rollback_unschedule: List[Tuple[Any, str, Any]] = []

        with self._lock:
            for directory in directories:
                self._release_directory_locked(
                    directory,
                    rollback_unschedule)

            self._remove_file_reference_locked(logical_path)

        self._flush_watch_operations([], rollback_unschedule)

        target, error = failures[0]

        if isinstance(error, OSError) and error.errno == errno.EMFILE:
            raise OSError(
                f"inotify instance or fd limit reached while observing "
                f"{target} ({diagnostics}). {hint}") from error

        raise OSError(
            f"cannot observe directory {target} for {logical_path}: {error}")

    def remove_file(self, path: str) -> None:
        """
        Stop observing a file or symlink.

        Missing paths are ignored. This makes cleanup operations idempotent.
        """
        logical_path = self._normalize_path(path)

        pending_unschedule: List[Tuple[Any, str, Any]] = []

        with self._lock:
            if logical_path not in self._file_refs:
                return

            Log.debug(
                f"{self.__class__.__name__}: stop observe path: "
                f"{logical_path}")

            path_info = self._path_info.get(logical_path)

            if path_info is None:
                # Inconsistent state: the file reference must still be dropped,
                # otherwise the path could never be released again.
                Log.debug(
                    f"{self.__class__.__name__}: no path info for "
                    f"{logical_path}, dropping reference only")
                directories: Tuple[str, ...] = ()
            else:
                _real_path, directories = path_info

            self._remove_file_reference_locked(logical_path)

            # Every add_file() call added one reference per directory, so one
            # reference per directory is released here.
            for directory in directories:
                self._release_directory_locked(
                    directory,
                    pending_unschedule)

        self._flush_watch_operations([], pending_unschedule)

    def _remove_file_reference_locked(self, logical_path: str) -> None:
        """
        Decrement a file reference and remove its mappings when unused.

        The lock must be held.
        """
        count = self._file_refs.get(logical_path, 0)

        if count <= 1:
            self._file_refs.pop(logical_path, None)
            self._last_event.pop(logical_path, None)
            self._pending_events.pop(logical_path, None)

            path_info = self._path_info.pop(logical_path, None)
            if path_info is not None:
                real_path, _directories = path_info
                aliases = self._real_paths.get(real_path)

                if aliases is not None:
                    aliases.discard(logical_path)
                    if not aliases:
                        self._real_paths.pop(real_path, None)
        else:
            self._file_refs[logical_path] = count - 1

    # -- launch files -----------------------------------------------------

    def register_launch(
        self,
        launch_file: str,
        paths: Iterable[str]
    ) -> List[str]:
        """
        Observe a launch file and all of its included files.

        Re-registering a launch file replaces its previous path set. Duplicate
        paths are collapsed, so every path is observed exactly once per launch
        file.

        :return:
            List of error messages for files that cannot be observed.
        """
        errors: List[str] = []
        added: List[str] = []

        # dict.fromkeys() removes duplicates and preserves the order.
        unique_paths = list(
            dict.fromkeys(
                self._normalize_path(path)
                for path in paths))

        for logical_path in unique_paths:
            try:
                self.add_file(logical_path)
                added.append(logical_path)
            except Exception as error:
                errors.append(f"{logical_path}: {error}")
                Log.error(
                    f"{self.__class__.__name__}: cannot observe "
                    f"{logical_path}: {error}")

        with self._lock:
            previous = self._launch_files.get(launch_file, ())

            # Update the reverse index: first drop the old assignment, then
            # add the new one. Paths contained in both sets stay assigned.
            for logical_path in previous:
                launch_files = self._launch_by_path.get(logical_path)

                if launch_files is not None:
                    launch_files.discard(launch_file)

                    if not launch_files:
                        self._launch_by_path.pop(logical_path, None)

            for logical_path in added:
                self._launch_by_path.setdefault(
                    logical_path, set()).add(launch_file)

            self._launch_files[launch_file] = tuple(added)

        # The references of the previous registration are released after the
        # new ones were added, so watches of unchanged files are never dropped.
        for logical_path in previous:
            try:
                self.remove_file(logical_path)
            except Exception:
                Log.error(
                    f"{self.__class__.__name__}: remove {logical_path} from "
                    f"observer failed:\n{traceback.format_exc()}")

        return errors

    def unregister_launch(self, launch_file: str) -> None:
        """
        Stop observing all files associated with a launch file.
        """
        with self._lock:
            paths = self._launch_files.pop(launch_file, ())

            for logical_path in paths:
                launch_files = self._launch_by_path.get(logical_path)

                if launch_files is not None:
                    launch_files.discard(launch_file)

                    if not launch_files:
                        self._launch_by_path.pop(logical_path, None)

        for logical_path in paths:
            try:
                self.remove_file(logical_path)
            except Exception:
                Log.error(
                    f"{self.__class__.__name__}: remove {logical_path} from "
                    f"observer failed:\n{traceback.format_exc()}")

    # -- queries ----------------------------------------------------------

    def is_observed(self, path: str) -> bool:
        """
        Return whether the logical path is currently observed.
        """
        logical_path = self._normalize_path(path)

        with self._lock:
            return logical_path in self._file_refs

    def affected_launch_files(self, path: str) -> FrozenSet[str]:
        """
        Return launch files that directly reference the given logical path.
        """
        logical_path = self._normalize_path(path)

        with self._lock:
            return self._affected_launch_files(logical_path)

    def _affected_launch_files(
        self,
        logical_path: str
    ) -> FrozenSet[str]:
        """
        Return affected launch files.

        The lock must be held. The lookup uses the reverse index and therefore
        does not depend on the number of registered launch files.
        """
        return frozenset(
            self._launch_by_path.get(logical_path, ()))

    def statistics(self) -> Dict[str, int]:
        """
        Return registry statistics.

        One scheduled watch corresponds to one inotify instance, because
        watchdog creates one emitter (and one inotify_init) per schedule().
        Directories inside a recursive watch root share a single watch, so
        'observed_directories' can be much larger than 'watches'.

        'directories' is kept as an alias of 'watches' for compatibility with
        older callers.
        """
        with self._lock:
            return {
                'files': len(self._file_refs),
                'file_references': sum(self._file_refs.values()),
                'aliases': sum(
                    len(paths)
                    for paths in self._real_paths.values()),
                'observed_directories': len(self._dir_refs),
                'watches': len(self._dir_watches),
                'directories': len(self._dir_watches),
                'watch_roots': len(self._watch_roots),
                'recursive_watches': sum(
                    1 for e in self._dir_watches.values() if e.recursive),
                'scheduled_watches': sum(
                    1 for e in self._dir_watches.values() if e.watch is not None),
                'launch_files': len(self._launch_files),
                'pending_events': len(self._pending_events),
                'queued_events': self._queue.qsize(),
                'inotify_max_user_instances': _read_inotify_limit('max_user_instances'),
                'inotify_max_user_watches': _read_inotify_limit('max_user_watches'),
            }

    def log_statistics(self) -> None:
        """
        Log a short summary of the current observation state.
        """
        stats = self.statistics()

        Log.info(
            f"{self.__class__.__name__}: files={stats['files']} "
            f"({stats['file_references']} refs), "
            f"observed directories={stats['observed_directories']}, "
            f"watches={stats['watches']} "
            f"(recursive={stats['recursive_watches']}), "
            f"launch files={stats['launch_files']}, "
            f"queued events={stats['queued_events']}, "
            f"inotify process={_count_inotify_instances()}, "
            f"user={_count_inotify_instances_of_user()}/"
            f"{stats['inotify_max_user_instances']}, "
            f"fds={_open_fd_count()}")

    # -- callback worker --------------------------------------------------

    def _run_worker(self) -> None:
        """
        Execute user callbacks and forward collapsed events.

        The worker decouples the watchdog dispatch thread from the callbacks.
        Blocking callbacks therefore never block the file system event
        processing and never hold the watchdog observer lock.
        """
        Log.debug(
            f"{self.__class__.__name__}: callback worker started")

        while True:
            timeout = self._next_pending_timeout()

            try:
                item = self._queue.get(timeout=timeout)
            except queue.Empty:
                self._flush_pending_events()
                continue

            if item is _SHUTDOWN:
                Log.debug(
                    f"{self.__class__.__name__}: callback worker stopped")
                return

            if item is _WAKEUP:
                self._flush_pending_events()
                continue

            self._invoke_callback(item)

    def _next_pending_timeout(self) -> Optional[float]:
        """
        Return the time until the next collapsed event has to be forwarded.

        :return:
            Seconds to wait or None if no collapsed event is pending.
        """
        with self._lock:
            if not self._pending_events:
                return None

            deadline = min(
                stamp
                for _event_type, stamp in self._pending_events.values())

        return max(0.0, deadline - time.monotonic())

    def _flush_pending_events(self) -> None:
        """
        Forward all collapsed events whose interval has elapsed.
        """
        callbacks: List[Tuple[str, str, FrozenSet[str]]] = []

        with self._lock:
            if not self._running:
                self._pending_events.clear()
                return

            now = time.monotonic()

            for logical_path in sorted(self._pending_events):
                event_type, deadline = self._pending_events[logical_path]

                if now < deadline:
                    continue

                del self._pending_events[logical_path]

                self._last_event[logical_path] = now

                callbacks.append(
                    (
                        event_type,
                        logical_path,
                        self._affected_launch_files(logical_path),
                    ))

        for item in callbacks:
            self._invoke_callback(item)

    def _invoke_callback(
        self,
        item: Tuple[str, str, FrozenSet[str]]
    ) -> None:
        """
        Call the change callback and log all exceptions.
        """
        event_type, logical_path, affected = item

        try:
            self._on_change(event_type, logical_path, affected)
        except Exception:
            Log.warn(
                f"{self.__class__.__name__}: change callback for "
                f"{logical_path} failed:\n{traceback.format_exc()}")

    def _enqueue(self, item: Any) -> None:
        """
        Hand one item over to the callback worker.
        """
        with self._lock:
            if self._worker is None:
                Log.debug(
                    f"{self.__class__.__name__}: no callback worker, "
                    f"dropping {item!r}")
                return

            self._queue.put(item)

    # -- event dispatching ------------------------------------------------

    def _dispatch(
        self,
        event_type: str,
        src_path: str,
        dest_path: Optional[str] = None,
        is_directory: bool = False
    ) -> None:
        """
        Dispatch a watchdog event.

        The watchdog thread calls this method. Recursive watch roots deliver
        events of unrelated files as well, those are discarded by the path
        resolution below.

        Path resolution and debouncing are performed while holding the internal
        lock. Watchdog operations and user callbacks are executed afterwards
        without the lock.
        """
        pending_schedule: List[Tuple[Any, str, int, bool]] = []
        pending_unschedule: List[Tuple[Any, str, Any]] = []
        callbacks: List[Tuple[str, str, FrozenSet[str]]] = []
        wake_worker = False

        with self._lock:
            if not self._running:
                return

            paths = self._resolve_paths_locked(
                src_path,
                is_directory,
                pending_schedule,
                pending_unschedule)

            if dest_path is not None:
                paths.update(
                    self._resolve_paths_locked(
                        dest_path,
                        is_directory,
                        pending_schedule,
                        pending_unschedule))

            now = time.monotonic()

            for logical_path in sorted(paths):
                last_event = self._last_event.get(logical_path)

                if (
                    last_event is not None
                    and self._min_event_interval > 0.0
                    and now - last_event < self._min_event_interval
                ):
                    # Collapse the event instead of dropping it, so the last
                    # change of a file is always reported (trailing edge).
                    self._pending_events[logical_path] = (
                        event_type,
                        last_event + self._min_event_interval,
                    )
                    wake_worker = True
                    continue

                self._last_event[logical_path] = now
                self._pending_events.pop(logical_path, None)

                callbacks.append(
                    (
                        event_type,
                        logical_path,
                        self._affected_launch_files(logical_path),
                    ))

        self._flush_watch_operations(
            pending_schedule,
            pending_unschedule)

        for item in callbacks:
            self._enqueue(item)

        if wake_worker:
            self._enqueue(_WAKEUP)

    def _refresh_path_locked(
        self,
        logical_path: str,
        pending_schedule: List[Tuple[Any, str, int, bool]],
        pending_unschedule: List[Tuple[Any, str, Any]]
    ) -> None:
        """
        Refresh the real target of a logical path.

        This is required when a symlink is replaced or retargeted. The lock
        must be held. Required watchdog calls are only recorded and executed by
        _flush_watch_operations() afterwards.
        """
        path_info = self._path_info.get(logical_path)
        reference_count = self._file_refs.get(logical_path, 0)

        if path_info is None or reference_count <= 0:
            return

        old_real_path, old_directories = path_info
        new_real_path, new_directories = self._path_details(logical_path)

        if (
            old_real_path == new_real_path
            and old_directories == new_directories
        ):
            return

        if not new_directories:
            # Nothing can be observed at the moment, for example because the
            # target directory was removed. The old watches are kept so that a
            # recreation of the path is still detected.
            Log.debug(
                f"{self.__class__.__name__}: no observable directory for "
                f"{logical_path}, keeping previous watches")
            return

        Log.debug(
            f"{self.__class__.__name__}: refresh path {logical_path}: "
            f"{old_real_path} -> {new_real_path}")

        for directory in new_directories:
            if directory in old_directories:
                continue

            self._reserve_directory_locked(
                directory,
                reference_count,
                pending_schedule)

        for directory in old_directories:
            if directory in new_directories:
                continue

            for _ in range(reference_count):
                self._release_directory_locked(
                    directory,
                    pending_unschedule)

        old_aliases = self._real_paths.get(old_real_path)

        if old_aliases is not None:
            old_aliases.discard(logical_path)

            if not old_aliases:
                self._real_paths.pop(old_real_path, None)

        self._real_paths.setdefault(new_real_path, set()).add(
            logical_path
        )

        self._path_info[logical_path] = (
            new_real_path,
            new_directories
        )

    def _resolve_paths_locked(
        self,
        event_path: str,
        is_directory: bool = False,
        pending_schedule: Optional[List[Tuple[Any, str, int, bool]]] = None,
        pending_unschedule: Optional[List[Tuple[Any, str, Any]]] = None
    ) -> Set[str]:
        """
        Resolve an event path to all matching logical observed paths.

        The lock must be held. Events of files that are not observed resolve to
        an empty set, which also filters the additional events delivered by
        recursive watch roots.

        Directory events are resolved to all observed files located directly
        inside that directory, which detects removal or renaming of a watched
        directory.
        """
        if pending_schedule is None:
            pending_schedule = []

        if pending_unschedule is None:
            pending_unschedule = []

        normalized_path = self._normalize_path(event_path)
        resolved_paths: Set[str] = set()

        # Refresh symlink targets when the logical path itself changed.
        if normalized_path in self._file_refs:
            self._refresh_path_locked(
                normalized_path,
                pending_schedule,
                pending_unschedule)
            resolved_paths.add(normalized_path)

        # Resolve events from the physical target directory.
        real_path = os.path.realpath(normalized_path)
        resolved_paths.update(
            self._real_paths.get(real_path, set())
        )

        if is_directory:
            # A deleted or moved directory affects all observed files inside.
            for observed_path, (path_real, _dirs) in self._path_info.items():
                if (
                    os.path.dirname(observed_path) == normalized_path
                    or os.path.dirname(path_real) == normalized_path
                ):
                    resolved_paths.add(observed_path)

        return resolved_paths

    def _resolve(self, src_path: str) -> Optional[str]:
        """
        Return one resolved logical path for compatibility with older callers.

        When several logical paths point to the same real file, the first
        sorted path is returned. Internal dispatch uses _resolve_paths_locked()
        and therefore notifies all aliases.
        """
        pending_schedule: List[Tuple[Any, str, int, bool]] = []
        pending_unschedule: List[Tuple[Any, str, Any]] = []

        with self._lock:
            paths = self._resolve_paths_locked(
                src_path,
                False,
                pending_schedule,
                pending_unschedule)

        self._flush_watch_operations(
            pending_schedule,
            pending_unschedule)

        if not paths:
            return None

        return sorted(paths)[0]
