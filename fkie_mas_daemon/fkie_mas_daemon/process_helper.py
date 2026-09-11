import os
import threading
import time
from typing import Dict, List, Optional, Tuple

# TTL for the cached /proc snapshot. Keep it short: process trees change fast.
PROC_TABLE_TTL = 1.0


class _ProcSnapshot:
    """Immutable snapshot of the process table plus a pre-built child index."""

    __slots__ = ('table', 'children_of', 'timestamp')

    def __init__(self, table: Dict[int, Tuple[int, str]], timestamp: float):
        self.table = table
        self.timestamp = timestamp
        children_of: Dict[int, List[int]] = {}
        for cpid, (ppid, _name) in table.items():
            children_of.setdefault(ppid, []).append(cpid)
        self.children_of = children_of


class ProcessHelper:

    def __init__(self):
        self._proc_snapshot: Optional[_ProcSnapshot] = None
        self._proc_lock = threading.Lock()

    # -- internals ---------------------------------------------------------

    @staticmethod
    def _read_proc_table() -> Dict[int, Tuple[int, str]]:
        """Read pid -> (ppid, name) for all processes in a single /proc sweep.

        Reads only /proc/<pid>/stat (one open+read per process, no psutil overhead).
        """
        table: Dict[int, Tuple[int, str]] = {}
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            try:
                with open(f'/proc/{entry}/stat', 'rb') as stat_file:
                    data = stat_file.read()
            except OSError:
                continue  # process vanished or not accessible
            try:
                # comm is wrapped in parentheses and may itself contain spaces/parens
                rpar = data.rindex(b')')
                name = data[data.index(b'(') + 1:rpar].decode('utf-8', 'replace')
                ppid = int(data[rpar + 2:].split(b' ', 3)[1])
            except (ValueError, IndexError):
                continue
            table[int(entry)] = (ppid, name)
        return table

    def _get_snapshot(self, max_age: float = PROC_TABLE_TTL,
                      force_refresh: bool = False) -> _ProcSnapshot:
        """Return a cached snapshot, refreshing it if older than max_age."""
        now = time.monotonic()
        snapshot = self._proc_snapshot
        if (not force_refresh and snapshot is not None
                and now - snapshot.timestamp <= max_age):
            snapshot.timestamp = now
            return snapshot

        with self._proc_lock:
            # re-check inside the lock: another thread may have refreshed already
            snapshot = self._proc_snapshot
            now = time.monotonic()
            if (not force_refresh and snapshot is not None
                    and now - snapshot.timestamp <= max_age):
                snapshot.timestamp = now
                return snapshot
            snapshot = _ProcSnapshot(self._read_proc_table(), time.monotonic())
            self._proc_snapshot = snapshot
            return snapshot

    def invalidate_proc_table(self) -> None:
        """Drop the cached snapshot, e.g. right after starting or killing a node."""
        with self._proc_lock:
            self._proc_snapshot = None

    # -- public API --------------------------------------------------------

    def get_child_pid(self, pid: int,
                      max_age: float = PROC_TABLE_TTL,
                      force_refresh: bool = False
                      ) -> Tuple[int, str, List[int]]:
        """Find the deepest descendant of `pid` (the real node process).

        Returns (found_pid, found_name, parents2kill) where parents2kill contains
        all intermediate pids between `pid` (exclusive) and found_pid (exclusive),
        e.g. the respawn wrapper and the shell started by screen.

        The underlying /proc snapshot is cached for `max_age` seconds, so calling
        this for many screens in a row costs only one sweep.
        """
        snapshot = self._get_snapshot(max_age=max_age, force_refresh=force_refresh)
        table = snapshot.table
        children_of = snapshot.children_of

        # If the pid is unknown the snapshot is likely stale -> retry once fresh.
        if pid not in table and not force_refresh:
            snapshot = self._get_snapshot(force_refresh=True)
            table = snapshot.table
            children_of = snapshot.children_of

        chain: List[int] = []
        current = pid
        # descend along the process chain; prefer the youngest (highest) pid on branches
        while True:
            kids = children_of.get(current)
            if not kids:
                break
            current = max(kids)
            chain.append(current)

        if not chain:
            return -1, '', []

        found_pid = chain[-1]
        return found_pid, table[found_pid][1], chain[:-1]
