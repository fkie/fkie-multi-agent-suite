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

import asyncio
from autobahn import wamp
import json
from fkie_mas_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_mas_pylib.crossbar.base_session import SelfEncoder
from fkie_mas_pylib.crossbar.runtime_interface import DaemonVersion
from . import version
from fkie_mas_pylib.logging.logging import Log


class VersionServicer(CrossbarBaseSession):
    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        realm: str = "ros",
        port: int = 11911,
        test_env=False,
    ):
        Log.info("Create version servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env=test_env)
        self._version, self._date = version.detect_version("fkie_mas_daemon")

    def stop(self):
        self.shutdown()

    @wamp.register("ros.daemon.get_version")
    def get_version(self) -> DaemonVersion:
        Log.info(f"{self.__class__.__name__}: get daemon version ")
        reply = DaemonVersion(self._version, self._date)
        return json.dumps(reply, cls=SelfEncoder)
