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
import asyncio
from autobahn import wamp

import os
import signal
from fkie_mas_pylib.interface.base_session import CrossbarBaseSession
from fkie_mas_pylib.interface.base_session import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import screen


class ScreenServicer(CrossbarBaseSession):

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 35685, test_env=False):
        Log.info("Create screen servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env=test_env)
        self._loaded_files = dict()  # dictionary of (CfgId: LaunchConfig)

    def stop(self):
        global IS_RUNNING
        IS_RUNNING = False
        self.shutdown()

    @wamp.register('ros.screen.kill_node')
    def killNode(self, name: str) -> bool:
        Log.info("Kill node '%s'", name)
        success = False
        screens = screen.get_active_screens(name)
        if len(screens.items()) == 0:
            return json.dumps({'result': success, 'message': 'Node does not have an active screen'}, cls=SelfEncoder)

        for session_name, node_name in screens.items():
            pid, session_name = screen.split_session_name(session_name)
            os.kill(pid, signal.SIGKILL)
            success = True
        return json.dumps({'result': success, 'message': ''}, cls=SelfEncoder)
