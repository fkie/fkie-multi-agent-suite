# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import json
import socket
import threading
import time
from types import SimpleNamespace
from typing import Any
from typing import Callable
from typing import List
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
        self._watchdog_thread = None
        self._last_ips = self._get_local_ips()
        self.register("subs", self._get_subscriptions)

    def shutdown(self):
        self._shutdown = True
        if (self._server):
            self._server.shutdown()
            self._server = None

    def _get_local_ips(self):
        ips = set()
        for iface in socket.getaddrinfo(socket.gethostname(), None):
            ip = iface[4][0]
            if not ip.startswith("127."):
                ips.add(ip)
        return ips

    def count_clients(self):
        with self._lock:
            return len(self._handler)

    def _cb_subs_changed(self, uri: str):
        count = 0
        with self._lock:
            for h in self._handler:
                if uri in h.subscriptions():
                    count += 1
        self.publish("event", {"type": "subs", "uri": uri, "count": count})

    def ws_handler(self, websocket) -> None:
        remote_address = websocket.remote_address
        print(f"add new ws client {remote_address}")
        handler = WebSocketHandler(self, websocket, self._cb_subs_changed)
        with self._lock:
            self._handler.add(handler)
        handler.spin()
        with self._lock:
            self._handler.remove(handler)
        for sub in handler.subscriptions():
            self._cb_subs_changed(sub)
        print(f"removed ws client {remote_address}")
        # TODO: remove all registered rpcs

    # blocking call
    def spin(self, port: int = 35430) -> None:
        while not self._shutdown:
            try:
                Log.info(f"Open Websocket on port {port}")
                with websockets.sync.server.serve(self.ws_handler, "0.0.0.0", port, max_size=2**22) as server:
                    self._server = server
                    self.port = port
                    self._last_ips = self._get_local_ips()
                    server.serve_forever()
            except Exception as e:
                import traceback
                print("Error while start websocket server: ", traceback.format_exc())
                time.sleep(2)
            finally:
                Log.info(f"Websocket on port {port} closed!")
                self._server = None
                for con in self._handler:
                    con.shutdown()

    def start_threaded(self, port: int = 35430) -> None:
        self._shutdown = False
        self._spin_thread = threading.Thread(
            target=self.spin, args=(port,), daemon=True)
        self._spin_thread.start()
        # Start Watchdog
        self._watchdog_thread = threading.Thread(
            target=self._watchdog, args=(port,), daemon=True)
        self._watchdog_thread.start()

    def _watchdog(self, port: int):
        while not self._shutdown:
            time.sleep(10)
            # check for interface change
            current_ips = self._get_local_ips()
            if current_ips != self._last_ips:
                print("network interface changed. Restart websocket server.")
                self._server.shutdown()
                self._last_ips = current_ips
            # Check if port is reachable
            try:
                with websockets.sync.client.connect(f"ws://127.0.0.1:{port}") as ws:
                    pass
            except Exception as e:
                print(f"port is not reachable: {e}. Restart websocket server.")
                if (self._server):
                    self._server.shutdown()
                    self._server = None

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

    def _get_subscriptions(self, uri) -> List[str]:
        with self._lock:
            subs = []
            for h in self._handler:
                if uri in h.subscriptions():
                    subs.append(h.address)
            return subs
