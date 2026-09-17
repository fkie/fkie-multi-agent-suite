# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import threading
import time

import psutil

import fkie_mas_daemon as nmd

# psutil reuses its Process objects, so cpu_percent() always reports the load
# since the previous call. To allow more than one sensor to use these values
# without splitting the measurement interval, the result is cached and shared
# between all callers.
_lock = threading.RLock()
_cache = []  # type: List[Tuple[float, str, int]]
_cache_ts = 0.0

# number of logical cores, used to normalize the per process load
CPU_COUNT = psutil.cpu_count() or 1


def process_load(max_age: float = 1.0, normalized: bool = False) -> list[tuple[float, str, int]]:
    """
    Returns the cpu load of all processes, sorted descending.

    :param float max_age: reuse the cached result if it is younger than this
                          value in seconds
    :param bool normalized: if True the load is divided by the count of logical
                            cores, so the value is comparable to a per core
                            threshold (psutil reports up to 100% per core)
    :return: list of (percent, name, pid)
    :rtype: list
    """
    global _cache, _cache_ts
    now = time.monotonic()
    with _lock:
        if _cache_ts > 0 and (now - _cache_ts) < max_age:
            # a recent measurement is available, reuse it
            result = list(_cache)
        else:
            result = []
            try:
                for proc in psutil.process_iter(attrs=["name", "cpu_percent"]):
                    try:
                        result.append((proc.info["cpu_percent"] or 0.0, proc.info["name"] or "", proc.pid))
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        # the process disappeared or is not accessible
                        continue
                result.sort(key=lambda item: item[0], reverse=True)
                _cache = result
                _cache_ts = now
            except Exception as err:
                nmd.ros_node.get_logger().debug("can not determine process load: %s" % err)
                # fall back to the last known values
                result = list(_cache)
    if normalized:
        return [(percent / CPU_COUNT, name, pid) for percent, name, pid in result]
    return result


def format_process_load(
    min_percent: float = 0.0, count: int = 3, normalized: bool = False, max_age: float = 1.0
) -> list[str]:
    """
    Returns a human readable list of the processes with the highest cpu load.

    :param float min_percent: ignore processes below this load
    :param int count: maximal count of returned entries
    :param bool normalized: see process_load()
    :param float max_age: see process_load()
    :rtype: list of str
    """
    result = []
    for percent, name, pid in process_load(max_age=max_age, normalized=normalized):
        # the list is sorted descending, so we can stop at the first miss
        if percent < min_percent:
            break
        result.append("%.2f%% %s [%d]" % (percent, name, pid))
        if len(result) >= count:
            break
    return result
