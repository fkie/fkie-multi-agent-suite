# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import json
import os
import signal
import threading
import time
import traceback
from typing import Dict, List, Optional, Tuple, Union

from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.runtime_interface import DelayRosUpdateState
from fkie_mas_pylib.interface.runtime_interface import ScreensMapping
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import process
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.websocket.server import WebSocketServer

from .process_helper import ProcessHelper

DEFAULT_KILL_SIGNAL = getattr(signal, "SIGKILL", signal.SIGTERM)


class ScreenServicer:

    def __init__(self, websocket: WebSocketServer):
        Log.info("Create ROS2 screen servicer")
        self._is_running = True
        self._process_helper = ProcessHelper()
        self._stop_event = threading.Event()
        self._ts_delay_until = 0.0
        self._screen_check_rate = 1.0
        self._screen_check_force_after_default = 10
        self._screen_check_force_after = self._screen_check_force_after_default
        self._screen_do_check = True
        self._screen_thread: Optional[threading.Thread] = None
        self._screen_thread_lock = threading.RLock()
        self._screens_set = set()
        self._screen_nodes_set = set()
        self._screen_json_msg: List[ScreensMapping] = []
        self._force_refresh = False
        self.websocket = websocket
        websocket.register("ros.screen.kill_node", self.kill_node)
        websocket.register("ros.screen.get_list", self.get_screen_list)
        websocket.subscribe("ros.daemon.delay_update_state", self.delay_update_state)
        self._thread_notify: Optional[threading.Timer] = None

    def start(self):
        if self._screen_thread is not None and self._screen_thread.is_alive():
            Log.debug(f"{self.__class__.__name__}: screen thread already running")
            return
        self._screen_thread = threading.Thread(target=self._check_screens, daemon=True)
        self._screen_thread.start()

    def stop(self):
        self._is_running = False
        self._stop_event.set()
        with self._screen_thread_lock:
            if self._thread_notify is not None:
                self._thread_notify.cancel()
                self._thread_notify = None

    def _send_update_notification(self):
        with self._screen_thread_lock:
            self._thread_notify = None
        if time.time() >= self._ts_delay_until:
            self.websocket.publish('ros.nodes.changed', {"timestamp": time.time()})

    def _check_screens(self):
        interval = 1.0 / self._screen_check_rate
        last_check = 0.0
        while self._is_running:
            if time.time() < self._ts_delay_until and not self._force_refresh:
                self._stop_event.wait(interval)
                continue
            if self._screen_do_check or last_check >= self._screen_check_force_after:
                screen.wipe()
                if self._screen_do_check:
                    # explicit request: reset backoff
                    self._screen_check_force_after = self._screen_check_force_after_default
                else:
                    # no request: exponential backoff, capped at 60s
                    self._screen_check_force_after = min(self._screen_check_force_after * 2, 60)
                self._screen_do_check = False

                # get screens
                screens = screen.get_active_screens()
                new_screens_set = set()
                new_screen_nodes_set = set()
                screen_dict: Dict[str, ScreensMapping] = {}
                for session_name, node_name in screens.items():
                    if node_name in screen_dict:
                        screen_dict[node_name].screens.append(session_name)
                    else:
                        screen_dict[node_name] = ScreensMapping(name=node_name, screens=[session_name])
                    new_screens_set.add(session_name)
                    new_screen_nodes_set.add(node_name)

                # publish the message only on difference
                with self._screen_thread_lock:
                    div_screen_nodes = self._screen_nodes_set ^ new_screen_nodes_set
                    div_screens = self._screens_set ^ new_screens_set
                    if div_screen_nodes or div_screens:
                        json_msg: List[ScreensMapping] = list(screen_dict.values())
                        # add nodes without screens sent by the last message
                        for node_name in self._screen_nodes_set - new_screen_nodes_set:
                            json_msg.append(ScreensMapping(name=node_name, screens=[]))
                        Log.debug(
                            f"{self.__class__.__name__}: publish ros.screen.list with {len(json_msg)} nodes.")
                        self.websocket.publish('ros.screen.list', {"screens": json_msg})
                        self._screen_json_msg = json_msg
                        self._screen_nodes_set = new_screen_nodes_set
                        self._screens_set = new_screens_set
                last_check = 0.0
                self._force_refresh = False
            else:
                last_check += interval
            # interruptible sleep
            self._stop_event.wait(interval)

    def system_change(self) -> None:
        self._screen_do_check = True

    def _resolve_signal(self, sig: Union[int, str, signal.Signals, None]) -> signal.Signals:
        """Convert a signal given as name, number or enum into signal.Signals."""
        if sig is None:
            return DEFAULT_KILL_SIGNAL
        if isinstance(sig, signal.Signals):
            return sig
        if isinstance(sig, str):
            try:
                return signal.Signals[sig.strip().upper()]
            except KeyError:
                pass
        elif isinstance(sig, int):
            try:
                return signal.Signals(sig)
            except ValueError:
                pass
        Log.warn(
            f"{self.__class__.__name__}: unknown signal '{sig}' provided. "
            f"Use default {DEFAULT_KILL_SIGNAL.name} instead")
        return DEFAULT_KILL_SIGNAL

    def kill_node(self, name: str, sig: Union[int, str, None] = None) -> str:
        sig_obj = self._resolve_signal(sig)
        Log.info(f"{self.__class__.__name__}: Kill node '{name}'; signal: {sig_obj.name}")
        success = False
        errors: List[str] = []

        screens = screen.get_active_screens(name)
        if not screens:
            return json.dumps(
                {'result': False, 'message': 'Node does not have an active screen'}, cls=SelfEncoder)

        for session_name, _node_name in screens.items():
            pid_screen, session_name = screen.split_session_name(session_name)
            found_pid, found_name, parents2kill = self._process_helper.get_child_pid(pid_screen)
            if found_pid > -1:
                try:
                    Log.debug(
                        f"{self.__class__.__name__}: Kill process '{found_name}' with process id "
                        f"'{found_pid}' using signal {sig_obj.name}")
                    os.kill(found_pid, sig_obj)
                    # kill all parents, to handle the case if respawn script is used
                    if sig_obj == DEFAULT_KILL_SIGNAL:
                        Log.debug(
                            f"{self.__class__.__name__}: Kill all parents '{parents2kill}' "
                            f"using signal {sig_obj.name}")
                        # keep only parents created after the screen process (do not kill the screen itself)
                        for parent_pid in sorted(parents2kill, reverse=True):
                            if parent_pid < pid_screen:
                                continue
                            try:
                                os.kill(parent_pid, sig_obj)
                            except ProcessLookupError:
                                pass
                    success_cur = True
                except Exception:
                    errors.append(traceback.format_exc())
            if not success_cur:
                try:
                    Log.debug(
                        f"{self.__class__.__name__}: Kill screen '{session_name}' with process id "
                        f"'{pid_screen}' using signal {sig_obj.name}")
                    os.kill(pid_screen, sig_obj)
                    success_cur = True
                    # print(f"kill screen time {time.time() - now}")
                except Exception:
                    errors.append(
                        f"Error while try to kill screen with session name "
                        f"'{session_name}': {traceback.format_exc()}")
            success = success or success_cur

        if success:
            with self._screen_thread_lock:
                if self._thread_notify is None:
                    self._thread_notify = threading.Timer(3.0, self._send_update_notification)
                    self._thread_notify.daemon = True
                    self._thread_notify.start()
        self._screen_do_check = True
        return json.dumps({'result': success, 'message': "\n".join(errors)}, cls=SelfEncoder)

    def get_screen_list(self, force: False) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.screen.get_list]")
        with self._screen_thread_lock:
            self._screen_do_check = True
            self._force_refresh = force
            return json.dumps(self._screen_json_msg, cls=SelfEncoder)

    def delay_update_state(self, delay_msg: DelayRosUpdateState) -> str:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.delay_update_state]: {delay_msg.sec} sec")
        if delay_msg.sec <= 0:
            return
        now = time.time()
        self._ts_delay_until = now + delay_msg.sec
