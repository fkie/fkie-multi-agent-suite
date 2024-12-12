# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import json
import os
import psutil
import signal
import threading
import time
from typing import Dict, List
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.runtime_interface import ScreensMapping
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.websocket.server import WebSocketServer
import fkie_mas_daemon as nmd


class ScreenServicer:

    def __init__(self, websocket: WebSocketServer):
        Log.info("Create ROS2 screen servicer")
        self._is_running = True
        self._screen_check_rate = 1
        self._screen_check_force_after_default = 10
        self._screen_check_force_after = self._screen_check_force_after_default
        self._screen_do_check = True
        self._screen_thread = None
        self._screens_set = set()
        self._screen_nodes_set = set()
        self._screen_json_msg: List[ScreensMapping] = []
        self.websocket = websocket
        websocket.register("ros.screen.kill_node", self.kill_node)
        websocket.register("ros.screen.get_list", self.get_screen_list)

    def start(self):
        self._screen_thread = threading.Thread(
            target=self._check_screens, daemon=True)
        self._screen_thread.start()

    def stop(self):
        self._is_running = False

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
                # create json message
                json_msg: List[ScreensMapping] = []
                for node_name, msg in screen_dict.items():
                    json_msg.append(msg)
                # add nodes without screens send by the last message
                gone_screen_nodes = self._screen_nodes_set - new_screen_nodes_set
                for sn in gone_screen_nodes:
                    json_msg.append(ScreensMapping(name=sn, screens=[]))

                # publish the message only on difference
                div_screen_nodes = self._screen_nodes_set ^ new_screen_nodes_set
                div_screens = self._screens_set ^ new_screens_set
                if div_screen_nodes or div_screens:
                    Log.debug(
                        f"{self.__class__.__name__}: publish ros.screen.list with {len(json_msg)} nodes.")
                    self.websocket.publish('ros.screen.list', json_msg)
                    self._screen_json_msg = json_msg
                    self._screen_nodes_set = new_screen_nodes_set
                    self._screens_set = new_screens_set
                last_check = 0
            else:
                last_check += 1
            time.sleep(1.0 / self._screen_check_rate)

    def system_change(self) -> None:
        self._screen_do_check = True

    def kill_node(self, name: str, sig: signal = signal.SIGKILL) -> bool:
        Log.info(f"{self.__class__.__name__}: Kill node '{name}'")
        self._screen_do_check = True
        success = False
        screens = screen.get_active_screens(name)
        if len(screens.items()) == 0:
            return json.dumps({'result': success, 'message': 'Node does not have an active screen'}, cls=SelfEncoder)
        for session_name, node_name in screens.items():
            successCur = False
            pid_screen, session_name = screen.split_session_name(session_name)
            # try to determine the process id of the node inside the screen
            found_deep = -1
            found_pid = -1
            found_name = ""
            try:
                for process in psutil.process_iter():
                    deep = -1
                    found = False
                    parents = process.parents()
                    # search for parents with screen process id
                    for p in parents:
                        deep += 1
                        if p.pid == pid_screen:
                            found = True
                            break
                    # use the process with most parents
                    if found and deep > found_deep:
                        found_deep = deep
                        found_pid = process.pid
                        found_name = process.name()
            except Exception as error:
                # fallback for psutil versions (<5.6.0) without Process.parents()
                current_pid = pid_screen
                new_pid = current_pid
                while new_pid == current_pid:
                    new_pid = -1
                    # search for process which has screen id as parent
                    for process in psutil.process_iter():
                        parent = process.parent()
                        if parent and parent.pid == current_pid:
                            new_pid = process.pid
                            current_pid = process.pid
                if current_pid != pid_screen:
                    found_pid = current_pid
            if found_pid > -1:
                Log.info(
                    f"{self.__class__.__name__}: Kill process '{found_name}' with process id '{found_pid}' using signal {sig.name}")
                os.kill(found_pid, sig)
                successCur = True
            if not successCur:
                Log.info(
                    f"{self.__class__.__name__}: Kill screen '{session_name}' with process id '{pid_screen}' using signal {sig.name}")
                os.kill(pid_screen, sig)
                successCur = True
            if successCur:
                success = True
        return json.dumps({'result': success, 'message': ''}, cls=SelfEncoder)

    def get_screen_list(self) -> str:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.screen.get_list]")
        return json.dumps(self._screen_json_msg, cls=SelfEncoder)
