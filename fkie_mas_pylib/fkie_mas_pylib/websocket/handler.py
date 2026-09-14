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
from concurrent.futures import ThreadPoolExecutor
from typing import Callable
from typing import List
from types import SimpleNamespace
import websockets
import websockets.sync.server
from inspect import signature
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.interface import SelfAllEncoder
from fkie_mas_pylib.websocket.queue import QueueItem, PQueue


class RemoteCallInfo:

    def __init__(self, origin_id: int, handler) -> None:
        self.origin_id = origin_id
        self.handler = handler


class WebSocketHandler:

    def __init__(self, server, connection: websockets.sync.server.ServerConnection, callback_on_sub: Callable[[str], None]):
        self._lock = threading.RLock()
        self.server = server
        self.connection = connection
        self._callback_on_sub = callback_on_sub
        # fallback address, the remote address may be unavailable already
        self.address = "unknown"
        try:
            self.address = f"{self.connection.remote_address[0]}:{self.connection.remote_address[1]}"
        except Exception:
            Log.debug(f"could not determine remote address: {traceback.format_exc()}")
        Log.info(f"{self.address}: connected")
        self._executor = ThreadPoolExecutor(max_workers=8, thread_name_prefix=f"ws-{self.address}")
        self._shutdown = False
        self.queue = PQueue(100, f"queue[{self.address}]")
        self._subscriptions = set()
        self._registrations = set()
        self._remote_calls_id = 0
        self._remote_calls_map = {}  # int: RemoteCallInfo
        self._send_thread = threading.Thread(
            target=self._send_handler, daemon=True)
        self._send_thread.start()

    def shutdown(self):
        self._shutdown = True
        self._executor.shutdown(wait=False)
        try:
            self.connection.close()
        except Exception as err:
            Log.debug(f"{self.address}: error while closing connection: {err}")

    def subscriptions(self) -> List[str]:
        # return a copy to avoid 'set changed size during iteration'
        with self._lock:
            return list(self._subscriptions)

    def has_subscription(self, uri: str) -> bool:
        '''
        :return: True if this client subscribed the given uri.
        '''
        with self._lock:
            return uri in self._subscriptions

    def _notify_subs_changed(self, uri: str) -> None:
        '''
        Informs the server about a changed subscription. Called without holding
        the handler lock to avoid deadlocks with other handlers.
        '''
        if self._callback_on_sub is None:
            return
        try:
            self._callback_on_sub(uri)
        except Exception as err:
            Log.warn(f"{self.address}: subscription callback for {uri} failed: {err}")

    def spin(self):
        try:
            for message in self.connection:
                try:
                    msg = json.loads(message,
                                     object_hook=lambda d: SimpleNamespace(**d))
                    has_id = hasattr(msg, 'id')
                    if not hasattr(msg, 'uri'):
                        # forward remote call
                        is_response = False
                        with self._lock:
                            if has_id:
                                is_response = msg.id in self._remote_calls_map.keys()
                        if is_response:
                            with self._lock:
                                # remove the entry, the call is finished
                                rci: RemoteCallInfo = self._remote_calls_map.pop(msg.id)
                            Log.info(
                                f'forward response {rci.origin_id} to {rci.handler.address}')
                            if hasattr(msg, 'result'):
                                reply = {"id": rci.origin_id,
                                         "result": msg.result}
                            elif hasattr(msg, 'error'):
                                reply = {"id": rci.origin_id,
                                         "error": msg.error}
                            else:
                                # neither result nor error, report a protocol error
                                reply = {"id": rci.origin_id,
                                         "error": "malformed response, neither result nor error"}
                            rci.handler.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfAllEncoder), priority=0))
                        else:
                            Log.warn(
                                f"[{self.address}]: received malformed message (without uri) {message}")
                            reply = {
                                "error": "malformed message, should contain uri"}
                            if has_id:
                                reply['id'] = msg.id
                            self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfAllEncoder), priority=0))
                        continue
                    if has_id:
                        # handle rpc calls
                        if msg.uri == 'sub':
                            # create subscription
                            changed = []
                            for uri in getattr(msg, 'params', []):
                                Log.info(f"[{self.address}]: add subscription to '{uri}'")
                                with self._lock:
                                    self._subscriptions.add(uri)
                                changed.append(uri)
                            reply = {"id": msg.id, "result": True}
                            self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfAllEncoder), priority=0))
                            # notify outside of the lock
                            for uri in changed:
                                self._notify_subs_changed(uri)
                        elif msg.uri == 'unsub':
                            # remove subscription
                            changed = []
                            for uri in getattr(msg, 'params', []):
                                Log.info(
                                    f"[{self.address}]: remove subscription to '{uri}'")
                                with self._lock:
                                    # discard() does not raise for unknown uris
                                    if uri in self._subscriptions:
                                        self._subscriptions.discard(uri)
                                        changed.append(uri)
                            reply = {"id": msg.id, "result": True}
                            self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfAllEncoder), priority=0))
                            # notify outside of the lock
                            for uri in changed:
                                self._notify_subs_changed(uri)
                        elif msg.uri == 'reg':
                            # register a method
                            for uri in getattr(msg, 'params', []):
                                self.server.register_rpc(uri, self)
                                with self._lock:
                                    self._registrations.add(uri)
                            reply = {"id": msg.id, "result": True}
                            self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfAllEncoder), priority=0))
                        else:
                            callback, local = self.server.get_callback(msg.uri)
                            params = msg.params if hasattr(msg, 'params') else []
                            if callback is not None:
                                if local:
                                    # call local method
                                    self._executor.submit(
                                        self.handle_callback,
                                        msg.id,
                                        callback,
                                        params
                                    )
                                else:
                                    # call rpc of a registered connected client
                                    Log.info(
                                        f"{self.address}: handle rpc for uri {msg.uri}, params: {params}")
                                    callback.remote_call(msg, self)
                            else:
                                Log.info(
                                    f"RPC-URI not found {msg.uri}, params: {params}")
                                reply = {
                                    "id": msg.id, "error": f"no method for {msg.uri} registered"}
                                self.queue.put(QueueItem(json.dumps(
                                    reply, cls=SelfAllEncoder), priority=0))
                    elif hasattr(msg, 'message'):
                        self.server.publish(msg.uri, msg.message)
                except Exception as error:
                    Log.warn(f"[{self.address}]: {error}: {traceback.format_exc()}")
        except websockets.ConnectionClosedError as close_error:
            self._shutdown = True
            Log.debug(f"{self.address}: {close_error}")
        except Exception:
            self._shutdown = True
            Log.warn(f"{self.address}: {traceback.format_exc()}")
        finally:
            self._shutdown = True
            Log.info(f"{self.address}: client removed")
            # copy the registrations to unregister them without holding the lock
            with self._lock:
                registrations = list(self._registrations)
            for reg in registrations:
                try:
                    self.server.unregister_rpc(reg, self)
                except Exception as err:
                    Log.debug(f"{self.address}: error while unregister {reg}: {err}")
            # wake up the send thread waiting on the queue
            try:
                self.queue.put(QueueItem('', priority=0))
            except Exception:
                pass

    def handle_callback(self, id, callback, args=None):
        Log.debug(f"{self.address}: handle callback {id}: {args}")
        # avoid a mutable default argument
        call_args = args if args is not None else []
        result = None
        error = None
        reply = ''
        try:
            sig = signature(callback)
            if ('requester' in sig.parameters):
                result = callback(*(arg for arg in call_args), requester=self.address)
            else:
                result = callback(*(arg for arg in call_args))
            if not isinstance(result, str):
                result = json.dumps(result, cls=SelfAllEncoder)
        except Exception as err:
            Log.warn(f"{self.address}: callback {id} failed: {traceback.format_exc()}")
            error = err
        if error is None:
            reply = f'{{"id": {id}, "result": {result}}}'
        else:
            # use json.dumps to escape quotes and newlines in the error message
            reply = f'{{"id": {id}, "error": {json.dumps(str(error))}}}'
        self.queue.put(QueueItem(reply, priority=0))

    # def handle_remote_callback(self, msg):
    #     Log.debug(f"{self.address}: handle remote callback {msg.uri}:{msg.id}: {msg}")
    #     return
    #     self.publish(msg)
    #     result = None
    #     error = None
    #     reply = ''
    #     try:
    #         result = callback(*(arg for arg in args))
    #         if not isinstance(result, str):
    #             result = json.dumps(result, cls=SelfAllEncoder)
    #     except Exception:
    #         import traceback
    #         error = traceback.format_exc()
    #     if error is None:
    #         reply = f'{{"id": {id}, "result": {result}}}'
    #     else:
    #         reply = f'{{"id": {id}, "error": {error}}}'
    #     self.queue.put(QueueItem(reply, priority=0))

    def publish(self, uri: str, message: str):
        # check the subscription under the lock, but queue without it
        if not self.has_subscription(uri):
            return
        self.queue.put(
            QueueItem(f'{{"uri": "{uri}", "message": {message}}}', priority=1))

    def remote_call(self, msg, handler):
        with self._lock:
            if msg.uri in self._registrations:
                Log.info(
                    f'forward call {msg.uri} to {self.address}, new id: {self._remote_calls_id}')
                self._remote_calls_map[self._remote_calls_id] = RemoteCallInfo(
                    msg.id, handler)
                msg.id = self._remote_calls_id
                self._remote_calls_id += 1
                self.queue.put(QueueItem(json.dumps(
                    msg, cls=SelfAllEncoder), priority=0))

    def _send_handler(self):
        try:
            while not self._shutdown:
                try:
                    item = self.queue.get()
                    if not item.data:
                        # empty item used as wake up signal on shutdown
                        continue
                    self.connection.send(item.data)
                except websockets.exceptions.ConnectionClosedOK:
                    # the client closed the connection, stop the loop
                    self._shutdown = True
                except websockets.exceptions.ConnectionClosed:
                    # avoid a busy loop on a broken connection
                    self._shutdown = True
                except Exception:
                    Log.info(traceback.format_exc())
        except Exception:
            Log.info(traceback.format_exc())
        finally:
            Log.debug(f"{self.address}: send handler stopped")
