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
import os
import signal
import threading
import time
from typing import Dict, List
from fkie_mas_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_mas_pylib.crossbar.base_session import SelfEncoder
from fkie_mas_pylib.crossbar.runtime_interface import ScreensMapping
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import screen
import fkie_mas_daemon as nmd


class ScreenServicer(CrossbarBaseSession):

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911):
        Log.info("Create ROS2 screen servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port)
        self._is_running = True
        self._screen_check_rate = 1
        self._screen_check_force_after_default = 10
        self._screen_check_force_after = self._screen_check_force_after_default
        self._screen_do_check = True
        self._screen_thread = None
        self._screens_set = set()
        self._screen_nodes_set = set()
        self._sceen_crossbar_msg: List(ScreensMapping) = []

    def start(self):
        self._screen_thread = threading.Thread(
            target=self._check_screens, daemon=True)
        self._screen_thread.start()

    def stop(self):
        self._is_running = False
        self.shutdown()

    def _check_screens(self):
        last_check = 0
        while self._is_running:
            if self._screen_do_check or last_check >= self._screen_check_force_after:
                screen.wipe()
                if self._screen_do_check:
                    self._screen_check_force_after = self._screen_check_force_after_default
                else:
                    self._screen_check_force_after += self._screen_check_force_after
                self._screen_do_check = False
                new_screens_set = set()
                new_screen_nodes_set = set()
                # get screens
                screens = screen.get_active_screens()
                screen_dict: Dict[str, ScreensMapping] = {}
                for session_name, node_name in screens.items():
                    if node_name in screen_dict:
                        screen_dict[node_name].screens.append(session_name)
                    else:
                        screen_dict[node_name] = ScreensMapping(
                            name=node_name, screens=[session_name])
                    new_screens_set.add(session_name)
                    new_screen_nodes_set.add(node_name)
                # create crossbar message
                crossbar_msg: List(ScreensMapping) = []
                for node_name, msg in screen_dict.items():
                    crossbar_msg.append(msg)
                # add nodes without screens send by the last message
                gone_screen_nodes = self._screen_nodes_set - new_screen_nodes_set
                for sn in gone_screen_nodes:
                    crossbar_msg.append(ScreensMapping(name=sn, screens=[]))

                # publish the message only on difference
                div_screen_nodes = self._screen_nodes_set ^ new_screen_nodes_set
                div_screens = self._screens_set ^ new_screens_set
                if div_screen_nodes or div_screens:
                    Log.debug(
                        f"{self.__class__.__name__}: publish ros.screen.list with {len(crossbar_msg)} nodes.")
                    self.publish_to('ros.screen.list', crossbar_msg)
                    self._screen_crossbar_msg = crossbar_msg
                    self._screen_nodes_set = new_screen_nodes_set
                    self._screens_set = new_screens_set
                last_check = 0
            else:
                last_check += 1
            time.sleep(1.0 / self._screen_check_rate)

    def system_change(self) -> None:
        self._screen_do_check = True

    @wamp.register('ros.screen.kill_node')
    def kill_node(self, name: str, sig: signal = signal.SIGKILL) -> bool:
        Log.info(f"{self.__class__.__name__}: Kill node '{name}'")
        self._screen_do_check = True
        success = False
        screens = screen.get_active_screens(name)
        if len(screens.items()) == 0:
            return json.dumps({'result': success, 'message': 'Node does not have an active screen'}, cls=SelfEncoder)
        for session_name, node_name in screens.items():
            pid, session_name = screen.split_session_name(session_name)
            os.kill(pid, sig)
            success = True
        return json.dumps({'result': success, 'message': ''}, cls=SelfEncoder)

    @wamp.register('ros.screen.get_list')
    def get_screen_list(self) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.screen.get_list]")
        return json.dumps(self._screen_crossbar_msg, cls=SelfEncoder)
