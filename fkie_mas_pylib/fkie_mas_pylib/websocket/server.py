# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import json
import threading
from types import SimpleNamespace
from typing import Any
from typing import Callable
from typing import Tuple
from typing import Union
import websockets
try:
    import websockets.sync
    import websockets.sync.server
except Exception as wse:
    try:
        import sys
        from importlib.metadata import version
        print(f"installed python3-websockets: {version('websockets')}, minimum required: 12.0", file=sys.stderr)
        print(f"please install: pip install \"websockets>=12.0\"", file=sys.stderr)
    except:
        raise wse
    exit(-1)

from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.interface import SelfAllEncoder
from fkie_mas_pylib.websocket.handler import WebSocketHandler


class WebSocketServer:

    def __init__(self):
        self._lock = threading.RLock()
        self._shutdown = False
        self._subscriptions = {}
        self._handler = set()
        self._registrations = {}
        self._remote_registrations = {}
        self._spin_thread = None
        self._server = None
        self.clients_count = 0
        self.port = -1

    def shutdown(self):
        self._shutdown = True
        if (self._server):
            self._server.shutdown()
            self._server = None

    def count_clients(self):
        with self._lock:
            return len(self._handler)

    def ws_handler(self, websocket) -> None:
        handler = WebSocketHandler(self, websocket)
        with self._lock:
            self._handler.add(handler)
        handler.spin()
        with self._lock:
            self._handler.remove(handler)
        # TODO: remove all registered rpcs

    # blocking call
    def spin(self, port: int=35430) -> None:
        Log.info(
            f"Open Websocket on port {port}")
        try:
            with websockets.sync.server.serve(self.ws_handler, "0.0.0.0", port, max_size=2**22) as server:
                self._server = server
                self.port = port
                server.serve_forever()
        except Exception:
            import traceback
            print("Error while start websocket server: ", traceback.format_exc())

    def start_threaded(self, port: int=35430) -> None:
        self._spin_thread = threading.Thread(
            target=self.spin, args=(port,), daemon=True)
        self._spin_thread.start()

    def subscribe(self, uri: str, callback: Callable[[Any], None]) -> None:
        with self._lock:
            Log.info(f"subscribe {uri}")
            self._subscriptions[uri] = callback

    def register(self, uri: str, callback: Callable[[int, ], None]) -> None:
        with self._lock:
            Log.info(f"register local callback for {uri}")
            self._registrations[uri] = callback

    def register_rpc(self, uri: str, handler: WebSocketHandler) -> None:
        with self._lock:
            Log.info(f"register remote {uri} @ {handler.address}")
            self._remote_registrations[uri] = handler

    def unregister_rpc(self, uri: str, handler: WebSocketHandler) -> None:
        with self._lock:
            try:
                if self._remote_registrations[uri] == handler:
                    Log.info(f"unregister remote {uri} @ {handler.address}")
                    del self._remote_registrations[uri]
            except:
                pass

    def publish(self, uri: str, message: Union[str, object]) -> bool:
        with self._lock:
            msg = message
            if not isinstance(msg, str):
                msg = json.dumps(msg, cls=SelfAllEncoder)
            for con in self._handler:
                con.publish(uri, msg)
            if uri in self._subscriptions:
                # forward to local subscriptions
                self._subscriptions[uri](json.loads(
                    msg, object_hook=lambda d: SimpleNamespace(**d)))

    def get_callback(self, uri: str) -> Tuple[Union[Callable[[int, ], None], None], bool]:
        with self._lock:
            if uri in self._registrations:
                return (self._registrations[uri], True)
            elif uri in self._remote_registrations:
                return (self._remote_registrations[uri], False)
        return (None, False)
