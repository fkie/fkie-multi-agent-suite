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
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket.server import WebSocketServer
from fkie_mas_daemon.rosstate_servicer import RosStateServicer
import fkie_mas_daemon as nmd
from . import version


class VersionServicer:
    def __init__(self, websocket: WebSocketServer, rosStateServicer: RosStateServicer):
        Log.info("Create ROS2 version servicer")
        self._version, self._date = version.detect_version(
            nmd.ros_node, "fkie_mas_daemon"
        )
        self.rosStateServicer = rosStateServicer
        websocket.register("ros.daemon.get_version", self.get_version)

    def stop(self):
        pass

    def get_version(self) -> DaemonVersion:
        self.rosStateServicer.publish_discovery_state()
        Log.info(f"{self.__class__.__name__}: get daemon version ")
        reply = DaemonVersion(self._version, self._date)
        return json.dumps(reply, cls=SelfEncoder)
