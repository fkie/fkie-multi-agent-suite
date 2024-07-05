# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import json
import threading
import websockets
import websockets.sync
import websockets.sync.server
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.websocket.handler import WebSocketHandler


class WebSocketServer:

    def __init__(self):
        self._shutdown = False
        self.subscriptions = {}
        self.handler = set()
        self.registrations = {}
        self.rpcs = {}
        self._spin_thread = None
        self._server = None

    def shutdown(self):
        self._shutdown = True
        if (self._server):
            self._server.shutdown()
            self._server = None

    def ws_handler(self, websocket):
        handler = WebSocketHandler(self, websocket)
        self.handler.add(handler)
        handler.spin()
        self.handler.remove(handler)
        # TODO: remove all registered rpcs

    def spin(self, port=35430):
        Log.info(
            f"Open Websocket on port {port}")
        try:
            with websockets.sync.server.serve(self.ws_handler, "0.0.0.0", port) as server:
                self._server = server
                server.serve_forever()
        except Exception:
            import traceback
            print("Error while start websocket server: ", traceback.format_exc())

    def start_threaded(self, port=35430):
        self._spin_thread = threading.Thread(
            target=self.spin, args=(port,), daemon=True)
        self._spin_thread.start()

    def subscribe(self, uri: str, callback):
        self.subscriptions[uri] = callback

    def register(self, uri: str, callback):
        self.registrations[uri] = callback

    def publish(self, uri: str, message: str) -> bool:
        message = message
        if not isinstance(message, str):
            message = json.dumps(message, cls=SelfEncoder)
        for con in self.handler:
            con.publish(uri, message)
