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
import websockets
import websockets.sync.server
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.interface import SelfAllEncoder
from fkie_mas_pylib.websocket.queue import QueueItem, PQueue


class RemoteCallInfo:

    def __init__(self, origin_id: int, handler) -> None:
        self.origin_id = origin_id
        self.handler = handler


class WebSocketHandler:

    def __init__(self, server, connection: websockets.sync.server.ServerConnection):
        self._lock = threading.RLock()
        self.server = server
        self.connection = connection
        try:
            self.address = f"{self.connection.remote_address[0]}:{self.connection.remote_address[1]}"
        except:
            import traceback
            print(traceback.format_exc())
        Log.info(f"{self.address}: connected")
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
                            rci: RemoteCallInfo = self._remote_calls_map[msg.id]
                            Log.info(
                                f'forward response {rci.origin_id} to {rci.handler.address}')
                            if hasattr(msg, 'result'):
                                reply = {"id": rci.origin_id,
                                         "result": msg.result}
                            elif hasattr(msg, 'error'):
                                reply = {"id": rci.origin_id,
                                         "error": msg.error}
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
                            Log.info(
                                f"[{self.address}]: add subscription to '{msg.params[0]}'")
                            with self._lock:
                                self._subscriptions.add(msg.params[0])
                            reply = {"id": msg.id, "result": True}
                            self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfAllEncoder), priority=0))
                        elif msg.uri == 'unsub':
                            # create subscription
                            try:
                                Log.info(
                                    f"[{self.address}]: remove subscription to '{msg.params[0]}'")
                                with self._lock:
                                    self._subscriptions.remove(msg.params[0])
                            except:
                                pass
                            reply = {"id": msg.id, "result": True}
                            self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfAllEncoder), priority=0))
                        elif msg.uri == 'reg':
                            # register a method
                            self.server.register_rpc(msg.params[0], self)
                            with self._lock:
                                self._registrations.add(msg.params[0])
                            reply = {"id": msg.id, "result": True}
                            self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfAllEncoder), priority=0))
                        else:
                            callback, local = self.server.get_callback(msg.uri)
                            if callback is not None:
                                if local:
                                    # call local method
                                    self.handle_callback(
                                        msg.id, callback, msg.params if hasattr(msg, 'params') else [])
                                else:
                                    # call rpc of a registered connected client
                                    Log.info(
                                        f"{self.address}: handle rpc for uri {msg.uri}, params: {msg.params}")
                                    callback.remote_call(msg, self)
                            else:
                                Log.info(
                                    f"rcp not found {msg.uri}, params: {msg.params}")
                                reply = {
                                    "id": msg.id, "error": f"no method for ${msg.uri} registered"}
                                self.queue.put(QueueItem(json.dumps(
                                    reply, cls=SelfAllEncoder), priority=0))
                    elif hasattr(msg, 'message'):
                        self.server.publish(msg.uri, msg.message)
                except Exception as error:
                    import traceback
                    print(traceback.format_exc())
                    Log.warn(f"[{self.address}]: {error}")
        except websockets.ConnectionClosedError as close_error:
            print(f"{self.address}: {close_error}")
        except Exception as error:
            import traceback
            print(traceback.format_exc())
        finally:
            Log.info(f"{self.address}: client removed")
            with self._lock:
                for reg in self._registrations:
                    self.server.unregister_rpc(reg, self)

    def handle_callback(self, id, callback, args=[]):
        Log.debug(f"{self.address}: handle callback {id}: {args}")
        result = None
        error = None
        reply = ''
        try:
            result = callback(*(arg for arg in args))
            if not isinstance(result, str):
                result = json.dumps(result, cls=SelfAllEncoder)
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            error = err
        if error is None:
            reply = f'{{"id": {id}, "result": {result}}}'
        else:
            reply = f'{{"id": {id}, "error": "{error}"}}'
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
        with self._lock:
            if uri in self._subscriptions:
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
                    self.connection.send(item.data)
                except Exception:
                    import traceback
                    Log.info(traceback.format_exc())
        except Exception:
            import traceback
            Log.info(traceback.format_exc())
        finally:
            pass
