# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import json
import threading
import traceback
from collections.abc import Callable
from types import SimpleNamespace
from typing import Any

import websockets
import websockets.sync
import websockets.sync.client

from fkie_mas_pylib.interface import SelfAllEncoder
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket.queue import PQueue, QueueItem


class WebSocketClient:
    """
    Helper class to communicate with daemon's websocket server.
    """

    # limits for the reconnect backoff
    MIN_RETRY_DELAY = 1.0
    MAX_RETRY_DELAY = 10.0
    OPEN_TIMEOUT = 5.0

    def __init__(self, port: int):
        self.port = port
        self._lock = threading.RLock()
        self._shutdown_event = threading.Event()
        self.queue = PQueue(100)
        self.subscriptions: dict[str, Callable[[Any], None]] = {}
        self.registrations: dict[str, Callable[..., Any]] = {}
        self.connection = None
        self._conn_try = 0
        self._msg_id = 0
        self._recv_thread = threading.Thread(target=self.recv_handler, daemon=True, name="mas-ws-client-recv")
        self._recv_thread.start()
        self._send_thread = threading.Thread(target=self._send_handler, daemon=True, name="mas-ws-client-send")
        self._send_thread.start()

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #

    def shutdown(self) -> None:
        """
        Closes the connection and stops the receive and send threads.
        """
        self._shutdown_event.set()
        with self._lock:
            connection, self.connection = self.connection, None
        if connection is not None:
            try:
                connection.close()
            except Exception as err:
                Log.debug(f"error while closing connection: {err}")
        # unblock the send thread which waits in queue.get()
        try:
            self.queue.put(QueueItem(None, priority=0))
        except Exception as err:
            Log.debug(f"could not enqueue shutdown sentinel: {err}")
        for thread in (self._recv_thread, self._send_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=3.0)
                if thread.is_alive():
                    Log.warn(f"thread {thread.name} did not stop within 3s")

    # ------------------------------------------------------------------ #
    # registration / subscription
    # ------------------------------------------------------------------ #

    def _next_msg_id(self) -> int:
        with self._lock:
            self._msg_id += 1
            return self._msg_id

    def _send_request(self, uri: str, param: str) -> None:
        """
        Sends a request to the server. The request is enqueued as is, it must
        not be wrapped into a publish message.
        """
        request = {"uri": uri, "id": self._next_msg_id(), "params": [param]}
        self.queue.put(QueueItem(json.dumps(request, cls=SelfAllEncoder), priority=0))

    def subscribe(self, uri: str, callback: Callable[[Any], None]) -> None:
        with self._lock:
            if uri in self.subscriptions:
                Log.warn(f"replace existing subscription for {uri}")
            self.subscriptions[uri] = callback
            connected = self.connection is not None
        if connected:
            Log.info(f"subscribe to {uri}")
            self._send_request("sub", uri)

    def register(self, uri: str, callback: Callable[..., Any]) -> None:
        with self._lock:
            if uri in self.registrations:
                Log.warn(f"replace existing callback for {uri}")
            self.registrations[uri] = callback
            connected = self.connection is not None
        if connected:
            Log.info(f"register callback for {uri}")
            self._send_request("reg", uri)

    def _send_initial_requests(self) -> None:
        """
        Sends all subscriptions and registrations after a (re)connect.
        """
        with self._lock:
            # copy the keys to avoid a RuntimeError if another thread subscribes
            subscriptions = list(self.subscriptions.keys())
            registrations = list(self.registrations.keys())
        for key in subscriptions:
            Log.info(f"subscribe to {key}")
            self._send_request("sub", key)
        for key in registrations:
            Log.info(f"register callback for {key}")
            self._send_request("reg", key)

    # ------------------------------------------------------------------ #
    # receiving
    # ------------------------------------------------------------------ #

    # blocking call
    def recv_handler(self) -> None:
        """
        Connects to the server and receives messages until :meth:`shutdown`
        is called. Reconnects with an exponential backoff.
        """
        retry_delay = self.MIN_RETRY_DELAY
        while not self._shutdown_event.is_set():
            # use 127.0.0.1: the server binds ipv4 only, "localhost" may resolve to ::1
            uri = f"ws://127.0.0.1:{self.port}"
            try:
                with websockets.sync.client.connect(uri, max_size=2**22, open_timeout=self.OPEN_TIMEOUT) as connection:
                    with self._lock:
                        self.connection = connection
                    self._conn_try = 0
                    retry_delay = self.MIN_RETRY_DELAY
                    Log.info(f"connected to {uri}, own address: {connection.local_address}")
                    self._send_initial_requests()
                    for message in connection:
                        self._handle_message(message)
            except KeyboardInterrupt:
                self._shutdown_event.set()
            except websockets.exceptions.ConnectionClosedOK:
                Log.debug(f"connection to {uri} closed")
            except websockets.exceptions.ConnectionClosedError as recv_error:
                Log.warn(f"connection to {uri} closed while recv: {recv_error}")
            except (ConnectionRefusedError, ConnectionResetError, TimeoutError, OSError) as cr_error:
                # log only the first attempt to avoid flooding the log
                if self._conn_try == 0:
                    Log.warn(f"{cr_error}")
            except Exception:
                if self._conn_try == 0:
                    Log.error(f"error in recv handler: {traceback.format_exc()}")
            finally:
                with self._lock:
                    was_connected = self.connection is not None
                    self.connection = None
                if was_connected:
                    Log.info("client disconnected")
                self._conn_try += 1
            # wait before reconnect, but return immediately on shutdown
            if self._shutdown_event.wait(retry_delay):
                break
            retry_delay = min(retry_delay * 2, self.MAX_RETRY_DELAY)

    def _handle_message(self, message: str | bytes) -> None:
        """
        Parses one received message and dispatches it to a subscription
        callback or to a registered rpc callback.
        """
        try:
            msg = json.loads(message, object_hook=lambda d: SimpleNamespace(**d))
            if not hasattr(msg, "uri"):
                if not hasattr(msg, "id"):
                    Log.warn(f"received malformed message (no uri, no id): {message}")
                # replies to own requests are not handled here
                return
            if hasattr(msg, "message"):
                with self._lock:
                    callback = self.subscriptions.get(msg.uri)
                if callback is None:
                    Log.debug(f"no subscription for {msg.uri}")
                    return
                callback(msg.message)
            elif hasattr(msg, "params"):
                with self._lock:
                    callback = self.registrations.get(msg.uri)
                if callback is None:
                    Log.warn(f"no callback registered for {msg.uri}")
                    return
                self.handle_callback(getattr(msg, "id", -1), callback, msg.params)
        except Exception as error:
            Log.warn(f"error while handling message: {error}")

    def handle_callback(self, msg_id: int, callback: Callable[..., Any], args=None) -> None:
        """
        Calls a registered callback and enqueues the reply.
        """
        if args is None:
            args = []
        Log.debug(f"handle callback {msg_id}: {args}")
        try:
            result = callback(*args)
            # a str result is expected to contain valid json already
            result_json = result if isinstance(result, str) else json.dumps(result, cls=SelfAllEncoder)
            reply = f'{{"id": {json.dumps(msg_id)}, "result": {result_json}}}'
        except Exception:
            # use json.dumps to escape quotes and newlines of the traceback
            reply = json.dumps({"id": msg_id, "error": traceback.format_exc()})
        self.queue.put(QueueItem(reply, priority=0))

    # ------------------------------------------------------------------ #
    # publishing
    # ------------------------------------------------------------------ #

    def publish(self, uri: str, message: str | object, latched: bool = False) -> None:
        """
        Enqueues a message for all subscribers of the given uri.

        :param message: a str is forwarded as is and must contain valid json.
        """
        # TODO: add resend_after_connect?
        msg = message if isinstance(message, str) else json.dumps(message, cls=SelfAllEncoder)
        item = {"uri": uri, "latched": latched}
        # insert the already serialized message without a second json round
        prefix = json.dumps(item, cls=SelfAllEncoder)[:-1]
        self.queue.put(QueueItem(f'{prefix}, "message": {msg}}}', priority=1))

    # blocking call
    def _send_handler(self) -> None:
        """
        Sends the queued messages until :meth:`shutdown` is called.
        """
        while not self._shutdown_event.is_set():
            try:
                item = self.queue.get()
                if item is None or item.data is None:
                    # shutdown sentinel
                    break
                with self._lock:
                    connection = self.connection
                if connection is None:
                    Log.debug("no connection, drop queued message")
                    continue
                connection.send(item.data)
            except websockets.exceptions.ConnectionClosedOK:
                pass
            except (
                websockets.exceptions.ConnectionClosedError,
                ConnectionRefusedError,
                ConnectionResetError,
                OSError,
            ) as send_error:
                Log.warn(f"websocket connection to port {self.port} lost while send: {send_error}")
                with self._lock:
                    self.connection = None
            except Exception:
                Log.error(f"error in send handler: {traceback.format_exc()}")
        Log.debug("send handler stopped")
