# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import json
import socket
import sys
import threading
import traceback
from collections.abc import Callable
from types import SimpleNamespace
from typing import Any

import websockets

try:
    import websockets.sync
    import websockets.sync.client
    import websockets.sync.server
except Exception as wse:
    try:
        from importlib.metadata import version

        installed_version = version("websockets")
    except Exception:
        raise wse
    print(f"installed python3-websockets: {installed_version}, minimum required: 12.0", file=sys.stderr)
    print('please install: pip install "websockets>=12.0"', file=sys.stderr)
    sys.exit(1)

from fkie_mas_pylib.interface import SelfAllEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket.handler import WebSocketHandler


class WebSocketServer:
    DEFAULT_PORT = 35430
    # limits for the reconnect backoff of the serve loop
    MIN_RETRY_DELAY = 1.0
    MAX_RETRY_DELAY = 30.0
    WATCHDOG_INTERVAL = 10.0

    def __init__(self):
        self._lock = threading.RLock()
        self._shutdown_event = threading.Event()
        self._subscriptions: dict[str, list[Callable[[Any], None]]] = {}
        self._handler: set[WebSocketHandler] = set()
        self._registrations: dict[str, Callable[..., Any]] = {}
        self._remote_registrations: dict[str, WebSocketHandler] = {}
        # listeners informed about changed subscription count: callback(uri, count)
        self._subscription_listeners: list[Callable[[str, int], None]] = []
        self._spin_thread: threading.Thread | None = None
        self._watchdog_thread: threading.Thread | None = None
        self._server = None
        self.port = -1
        self._last_ips = self._get_local_ips()
        self.register("subs", self._get_subscriptions)

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #

    def start_threaded(self, port: int = DEFAULT_PORT, watchdog: bool = False) -> None:
        """
        Starts the websocket server in a background thread.

        :param port: the tcp port to listen on.
        :param watchdog: if True a watchdog restarts the server on network changes.
        """
        with self._lock:
            if self._spin_thread is not None and self._spin_thread.is_alive():
                Log.warn("websocket server already running, ignore start_threaded()")
                return
            self._shutdown_event.clear()
            self._spin_thread = threading.Thread(target=self.spin, args=(port,), daemon=True, name="mas-ws-server")
            self._spin_thread.start()
            if watchdog:
                self._watchdog_thread = threading.Thread(
                    target=self._watchdog, args=(port,), daemon=True, name="mas-ws-watchdog"
                )
                self._watchdog_thread.start()

    # blocking call
    def spin(self, port: int = DEFAULT_PORT) -> None:
        """
        Runs the websocket server until :meth:`shutdown` is called.
        Reconnects with an exponential backoff on errors.
        """
        retry_delay = self.MIN_RETRY_DELAY
        while not self._shutdown_event.is_set():
            try:
                Log.info(f"Open Websocket on port {port}")
                with websockets.sync.server.serve(
                    self.ws_handler, "0.0.0.0", port, max_size=2**22, compression=None
                ) as server:
                    with self._lock:
                        self._server = server
                        self.port = port
                    self._last_ips = self._get_local_ips()
                    # reset backoff after a successful bind
                    retry_delay = self.MIN_RETRY_DELAY
                    server.serve_forever()
                Log.info(f"Websocket on port {port} closed!")
            except Exception:
                Log.error(f"Error while starting websocket server: {traceback.format_exc()}")
                # wait, but return immediately on shutdown
                self._shutdown_event.wait(retry_delay)
                retry_delay = min(retry_delay * 2, self.MAX_RETRY_DELAY)
            finally:
                with self._lock:
                    self._server = None
                    self.port = -1
                    # copy to avoid mutation while handler threads are ending
                    handlers = list(self._handler)
                for con in handlers:
                    try:
                        con.shutdown()
                    except Exception as err:
                        Log.debug(f"error while shutdown handler: {err}")

    def shutdown(self) -> None:
        """
        Stops the server and closes all client connections.
        """
        self._shutdown_event.set()
        with self._lock:
            server, self._server = self._server, None
            handlers = list(self._handler)
        if server is not None:
            try:
                server.shutdown()
            except Exception as err:
                Log.debug(f"error while shutdown server: {err}")
        for con in handlers:
            try:
                con.shutdown()
            except Exception as err:
                Log.debug(f"error while shutdown handler: {err}")
        thread = self._spin_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.0)
            if thread.is_alive():
                Log.warn("websocket server thread did not stop within 5s")
        self._spin_thread = None

    # ------------------------------------------------------------------ #
    # client handling
    # ------------------------------------------------------------------ #

    def ws_handler(self, websocket) -> None:
        """
        Handles one client connection. Called by the websocket server in an own thread.
        """
        remote_address = websocket.remote_address
        Log.debug(f"add new ws client {remote_address}")
        handler = WebSocketHandler(self, websocket, self._cb_subs_changed)
        with self._lock:
            self._handler.add(handler)
        try:
            handler.spin()
        except Exception:
            Log.warn(f"ws client {remote_address} terminated with error: {traceback.format_exc()}")
        finally:
            subscriptions = []
            try:
                subscriptions = list(handler.subscriptions())
            except Exception as err:
                Log.debug(f"could not read subscriptions of {remote_address}: {err}")
            with self._lock:
                self._handler.discard(handler)
                # remove all rpcs registered by this client
                stale = [uri for uri, h in self._remote_registrations.items() if h is handler]
                for uri in stale:
                    Log.info(f"unregister remote {uri} @ {remote_address}")
                    del self._remote_registrations[uri]
            for uri in subscriptions:
                self._cb_subs_changed(uri)
            Log.debug(f"removed ws client {remote_address}")

    def count_clients(self) -> int:
        with self._lock:
            return len(self._handler)

    @property
    def clients_count(self) -> int:
        # kept for backward compatibility
        return self.count_clients()

    def _cb_subs_changed(self, uri: str) -> None:
        """
        Called by a handler if a client subscribed or unsubscribed and on
        client disconnect.
        """
        count = self.subscriptions(uri)
        self.publish("event", {"type": "subs", "uri": uri, "count": count})
        with self._lock:
            listeners = list(self._subscription_listeners)
        for clb in listeners:
            try:
                clb(uri, count)
            except Exception as err:
                Log.warn(f"subscription listener for {uri} failed: {err}")

    # ------------------------------------------------------------------ #
    # registration / subscription
    # ------------------------------------------------------------------ #

    def subscribe(self, uri: str, callback: Callable[[Any], None]) -> None:
        with self._lock:
            Log.info(f"subscribe {uri}")
            if uri not in self._subscriptions:
                self._subscriptions[uri] = []
            self._subscriptions[uri].append(callback)
        # a local subscription also changes the subscriber count
        self._cb_subs_changed(uri)

    def unsubscribe(self, uri: str, callback: Callable[[Any], None]) -> None:
        """
        Removes a local subscription added by :meth:`subscribe`.
        """
        removed = False
        with self._lock:
            callbacks = self._subscriptions.get(uri)
            if callbacks is not None and callback in callbacks:
                callbacks.remove(callback)
                removed = True
                if not callbacks:
                    del self._subscriptions[uri]
        if removed:
            self._cb_subs_changed(uri)

    def subscriptions(self, uri: str) -> int:
        """
        Returns the count of subscribers for the given uri. Remote clients as
        well as local callbacks registered by :meth:`subscribe` are counted.

        :param uri: the uri to check, e.g. 'ros.provider.system_diagnostics'
        :return: count of subscribers, 0 if nobody is listening.
        """
        with self._lock:
            handlers = list(self._handler)
            # local subscriptions are counted, too
            count = len(self._subscriptions.get(uri, []))
        for h in handlers:
            try:
                if h.has_subscription(uri):
                    count += 1
            except Exception:
                # handler is currently shutting down
                continue
        return count

    def has_subscriptions(self, uri: str) -> bool:
        """
        :return: True if at least one subscriber exists for the given uri.
        """
        return self.subscriptions(uri) > 0

    def add_subscription_listener(self, callback: Callable[[str, int], None]) -> None:
        """
        Adds a listener which is called with (uri, count) if the count of
        subscribers changed. Allows event driven start/stop of producers.
        """
        with self._lock:
            if callback not in self._subscription_listeners:
                self._subscription_listeners.append(callback)

    def remove_subscription_listener(self, callback: Callable[[str, int], None]) -> None:
        with self._lock:
            if callback in self._subscription_listeners:
                self._subscription_listeners.remove(callback)

    def register(self, uri: str, callback: Callable[..., Any]) -> None:
        with self._lock:
            if uri in self._registrations:
                Log.warn(f"replace existing local callback for {uri}")
            Log.info(f"register local callback for {uri}")
            self._registrations[uri] = callback

    def register_rpc(self, uri: str, handler: WebSocketHandler) -> None:
        with self._lock:
            if uri in self._remote_registrations:
                Log.warn(f"replace existing remote registration for {uri}")
            Log.info(f"register remote {uri} @ {handler.address}")
            self._remote_registrations[uri] = handler

    def unregister_rpc(self, uri: str, handler: WebSocketHandler) -> None:
        with self._lock:
            if self._remote_registrations.get(uri) is handler:
                Log.info(f"unregister remote {uri} @ {handler.address}")
                del self._remote_registrations[uri]

    def get_callback(self, uri: str) -> tuple[Callable[..., Any] | WebSocketHandler | None, bool]:
        """
        :return: (callback or remote handler, True if it is a local callback)
        """
        with self._lock:
            if uri in self._registrations:
                return (self._registrations[uri], True)
            if uri in self._remote_registrations:
                return (self._remote_registrations[uri], False)
        return (None, False)

    def _get_subscriptions(self, uri: str) -> list[str]:
        """
        RPC 'subs': returns the addresses of all clients subscribed to the uri.
        Local subscriptions are reported as 'local'.
        """
        with self._lock:
            handlers = list(self._handler)
            local_count = len(self._subscriptions.get(uri, []))
        subs = ["local"] * local_count
        for h in handlers:
            try:
                if h.has_subscription(uri):
                    subs.append(h.address)
            except Exception:
                continue
        return subs

    # ------------------------------------------------------------------ #
    # publishing
    # ------------------------------------------------------------------ #

    def publish(self, uri: str, message: str | object) -> bool:
        """
        Sends the message to all connected clients and to local subscribers.

        :return: True if the message was delivered to all destinations.
        """
        # do not hold the lock while doing network io
        with self._lock:
            handlers = list(self._handler)
            local_callbacks = self._subscriptions.get(uri)
        # skip serialization if nobody is subscribed
        if not local_callbacks and not any(self._has_subscription_quiet(con, uri) for con in handlers):
            return True
        try:
            msg = message if isinstance(message, str) else json.dumps(message, cls=SelfAllEncoder)
        except Exception as err:
            Log.warn(f"could not serialize message for {uri}: {err}")
            return False
        result = True
        for con in handlers:
            try:
                con.publish(uri, msg)
            except Exception as err:
                result = False
                Log.debug(f"publish {uri} to {con.address} failed: {err}")
        if local_callbacks is not None:
            try:
                # forward to local subscription; avoid a second serialization round
                payload = (
                    message
                    if not isinstance(message, str)
                    else json.loads(msg, object_hook=lambda d: SimpleNamespace(**d))
                )
                for clb in local_callbacks:
                    clb(payload)
            except Exception as err:
                result = False
                Log.warn(f"local subscription for {uri} failed: {err}")
        return result

    @staticmethod
    def _has_subscription_quiet(handler: WebSocketHandler, uri: str) -> bool:
        """
        Exception safe check used while publishing.
        """
        try:
            return handler.has_subscription(uri)
        except Exception:
            return False

    # ------------------------------------------------------------------ #
    # helper / watchdog
    # ------------------------------------------------------------------ #

    @staticmethod
    def _get_local_ips() -> set[str]:
        ips: set[str] = set()
        try:
            for iface in socket.getaddrinfo(socket.gethostname(), None):
                ip = iface[4][0]
                # strip scope id of link local ipv6 addresses
                plain_ip = ip.split("%")[0]
                if plain_ip.startswith("127.") or plain_ip == "::1":
                    continue
                ips.add(plain_ip)
        except OSError as err:
            Log.warn(f"could not resolve local ip addresses: {err}")
        return ips

    def _watchdog(self, port: int) -> None:
        """
        Restarts the server if the network interfaces changed or the port is
        not reachable anymore.
        """
        while not self._shutdown_event.is_set():
            if self._shutdown_event.wait(self.WATCHDOG_INTERVAL):
                return
            # check for interface changes
            current_ips = self._get_local_ips()
            if current_ips != self._last_ips:
                Log.info("network interface changed. Restart websocket server.")
                self._last_ips = current_ips
                self._restart_server()
                continue
            # check if the port is still reachable
            try:
                with websockets.sync.client.connect(f"ws://127.0.0.1:{port}", open_timeout=5) as ws:
                    ws.close()
            except Exception as err:
                Log.warn(f"port is not reachable: {err}. Restart websocket server.")
                self._restart_server()

    def _restart_server(self) -> None:
        """
        Stops the current server instance; the serve loop in :meth:`spin` reopens it.
        """
        with self._lock:
            server = self._server
        if server is not None:
            try:
                server.shutdown()
            except Exception as err:
                Log.debug(f"error while restarting server: {err}")
