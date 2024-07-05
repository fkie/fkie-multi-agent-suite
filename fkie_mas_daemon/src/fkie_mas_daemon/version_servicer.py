# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import json
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.runtime_interface import DaemonVersion
from . import version
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket.server import WebSocketServer


class VersionServicer:
    def __init__(
        self,
        websocket: WebSocketServer,
        test_env=False,
    ):
        Log.info("Create version servicer")
        self._version, self._date = version.detect_version("fkie_mas_daemon")
        websocket.register("ros.daemon.get_version", self.get_version)

    def stop(self):
        pass

    def get_version(self) -> DaemonVersion:
        Log.info(f"{self.__class__.__name__}: get daemon version ")
        reply = DaemonVersion(self._version, self._date)
        return json.dumps(reply, cls=SelfEncoder)
