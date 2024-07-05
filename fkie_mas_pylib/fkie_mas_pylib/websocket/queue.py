# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import threading
from typing import Union
from fkie_mas_pylib.logging.logging import Log


class Full(Exception):
    pass


class QueueItem:

    def __init__(self, data: str, priority: int=1):
        self.data = data
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority


class PQueue(object):

    def __init__(self, maxsize: int=0, logger_name: str='queue'):
        '''
        :param int maxsize: The maximal queue length for each priority. No new items are added if this size is reached. Zero to disable the limit for each priority.
        :param str logger_name: the name of this priority queue used for logging or exceptions.
        '''
        self._logger_name = logger_name
        self._cv = threading.Condition()
        self._maxsize = maxsize
        self._pq = {2: [],
                    1: [],
                    0: []}
        self._counts = {2: 0,
                        1: 0,
                        0: 0}
        self._idx = [2, 1, 0]
        self._count = 0

    def clear(self) -> None:
        Log.debug(f"Queue {self._logger_name}: clear queue")
        with self._cv:
            for idx in self._idx:
                del self._pq[idx][:]
                self._count = 0
                self._counts[idx] = 0
            self._cv.notify()

    def put(self, item: QueueItem) -> None:
        if self._maxsize > 0 and self._counts[item.priority] >= self._maxsize:
            raise Full(
                f"Queue `{self._logger_name}` for priority {item.priority} is full")
        with self._cv:
            Log.debug("add %s" % item)
            self._pq[item.priority].append(item)
            self._count += 1
            self._counts[item.priority] += 1
            self._cv.notify()

    def get(self, block=True) -> Union[QueueItem, None]:
        try:
            with self._cv:
                if self.size() == 0:
                    if block:
                        self._cv.wait()
                if self.size() > 0:
                    item = None
                    for idx in self._idx:
                        if self._counts[idx]:
                            item = self._pq[idx].pop(0)
                            self._count -= 1
                            self._counts[item.priority] -= 1
                            Log.debug(
                                f"Queue {self._logger_name}: get {item}")
                            return item
            return None
        except Exception:
            import traceback
            print(traceback.format_exc())

    def size(self, priority: Union[int, None]=None) -> int:
        if priority is None:
            return self._count
        if priority in self._counts:
            return self._counts[priority]
        return 0
