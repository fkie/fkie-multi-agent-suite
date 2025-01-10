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
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.websocket.server import WebSocketServer


class ScreenServicer:

    def __init__(self, websocket:WebSocketServer, test_env=False):
        Log.info("Create screen servicer")
        self._loaded_files = dict()  # dictionary of (CfgId: LaunchConfig)
        websocket.register("ros.screen.kill_node", self.killNode)

    def stop(self):
        pass

    def killNode(self, name: str, sig: signal = signal.SIGKILL) -> bool:
        Log.info(f"{self.__class__.__name__}: Kill node '{name}'; signal: [{type(sig)}] {sig}")
        sig_obj = sig
        if isinstance(sig, str):
            if sig == 'SIGTERM':
                sig_obj = signal.SIGTERM
            elif sig == 'SIGKILL':
                sig_obj = signal.SIGTERM
            else:
                Log.warn(f"{self.__class__.__name__}:  unknown signal '{sig}' provided. Use default SIGKILL instead")
                sig_obj = signal.SIGKILL
        success = False
        screens = screen.get_active_screens(name)
        if len(screens.items()) == 0:
            return json.dumps({'result': success, 'message': 'Node does not have an active screen'}, cls=SelfEncoder)

        for session_name, node_name in screens.items():
            pid, session_name = screen.split_session_name(session_name)
            os.kill(pid, sig_obj)
            success = True
        return json.dumps({'result': success, 'message': ''}, cls=SelfEncoder)
