# The MIT License (MIT)

# Copyright (c) 2014-2024 Fraunhofer FKIE, Alexander Tiderko

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import json
import os


class RosPackage:
    def __init__(self, name: str, path: str) -> None:
        self.name = name
        self.path = path

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class PathItem:
    """
    :param str path: absolute path of the file or directory
    :param float mtime: time of last modification of path. The return value is a number giving the number of seconds since the epoch
    :param int size: size, in bytes, of path
    :param str path_type: one of types {file, dir, symlink, package}
    """

    def __init__(self, path: str, mtime: float, size: int, path_type: str) -> None:
        self.path = path
        self.mtime = mtime
        self.size = size
        self.type = path_type

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LogPathItem:
    """
    :param str node: complete node name
    :param str screen_log: the absolute path to the screen log file.
    :param bool screen_log_exists: False if the file does not exists.
    :param str ros_log: the absolute path to the ros log file.
    :param bool ros_log_exists: False if the file does not exists.
    """

    def __init__(
        self,
        node: str,
        screen_log: str = "",
        screen_log_exists: bool = False,
        ros_log: str = "",
        ros_log_exists: bool = False,
    ) -> None:
        self.node = node
        self.screen_log = screen_log
        self.screen_log_exists = screen_log_exists
        self.ros_log = ros_log
        self.ros_log_exists = ros_log_exists

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LogPathClearResult:
    """
    :param str node: complete node name
    :param bool result: Clear result.
    :param str message: Message on error.
    """

    def __init__(
        self,
        node: str,
        result: bool = False,
        message: str = "",
    ) -> None:
        self.node = node
        self.result = result
        self.message = message

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)

class FileItem:
    """
    :param str path: absolute path of the file or directory
    :param float mtime: time of last modification of path. The return value is a number giving the number of seconds since the epoch
    :param int size: size, in bytes, of path
    :param str value: content of the file
    """

    def __init__(
        self,
        path: str,
        mtime: float = 0,
        size: int = 0,
        value: str = "",
        encoding="utf-8",
    ) -> None:
        self.path = path
        self.fileName = os.path.split(path)[-1]
        self.mtime = mtime
        self.size = size
        self.extension = path.rsplit(".", 1)[-1]
        self.value = value
        self.encoding = encoding

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)
