# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import json
import threading
import time
from types import SimpleNamespace
from typing import Any
from typing import Callable
from typing import Union
import websockets
import websockets.sync
import websockets.sync.client
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.websocket.queue import QueueItem, PQueue


class WebSocketClient:
    '''
    Helper class to communicate with daemon's websocket server.
    '''

    def __init__(self, port: int):
        self.port = port
        self._shutdown = False
        self.queue = PQueue(100)
        self.subscriptions = {}
        self.registrations = {}
        self.connection = None
        self._conn_try = 0
        self._msg_id = 0
        self._recv_thread = threading.Thread(
            target=self.recv_handler, daemon=True)
        self._recv_thread.start()
        self._send_thread = threading.Thread(
            target=self._send_handler, daemon=True)
        self._send_thread.start()

    def shutdown(self) -> None:
        self._shutdown = True

    def subscribe(self, uri: str, callback: Callable[[Any], None]) -> None:
        self.subscriptions[uri] = callback
        if self.connection:
            Log.info(f"subscribe to {uri}")
            # send subscription
            self._msg_id += 1
            request = {"uri": "sub", "id": self._msg_id, "params": [uri]}
            self.publish("sub", json.dumps(request))

    def register(self, uri: str, callback: Callable[[int, ], None]) -> None:
        self.registrations[uri] = callback
        if self.connection:
            Log.info(f"register callback for {uri}")
            # send registration
            self._msg_id += 1
            request = {"uri": "reg", "id": self._msg_id, "params": [uri]}
            self.publish("reg", json.dumps(request))

    # blocking call
    def recv_handler(self) -> None:
        while not self._shutdown:
            try:
                uri = f"ws://localhost:{self.port}"
                with websockets.sync.client.connect(uri, max_size=2**22) as connection:
                    try:
                        self._conn_try = 0
                        Log.info(
                            f"connected to {uri}, own address: {connection.local_address}")
                        self.connection = connection
                        # send subscriptions
                        for key in self.subscriptions.keys():
                            Log.info(f"subscribe to {key}")
                            self._msg_id += 1
                            request = {"uri": "sub",
                                       "id": self._msg_id, "params": [key]}
                            self.queue.put(QueueItem(json.dumps(
                                request, cls=SelfEncoder), priority=0))
                        # send registrations
                        for key in self.registrations.keys():
                            Log.info(f"register callback for {key}")
                            self._msg_id += 1
                            request = {"uri": "reg",
                                       "id": self._msg_id, "params": [key]}
                            self.queue.put(QueueItem(json.dumps(
                                request, cls=SelfEncoder), priority=0))
                        for message in connection:
                            try:
                                msg = json.loads(message,
                                                 object_hook=lambda d: SimpleNamespace(**d))
                                if not hasattr(msg, 'uri') and not hasattr(msg, 'id'):
                                    Log.warn(
                                        f"[{self.connection}]: received malformed message (without uri and id) {message}")
                                else:
                                    if hasattr(msg, 'message'):
                                        if msg.uri in self.subscriptions:
                                            self.subscriptions[msg.uri](
                                                msg.message)
                                    if hasattr(msg, 'params'):
                                        self.handle_callback(
                                            msg.id, self.registrations[msg.uri], msg.params if hasattr(msg, 'params') else [])
                            except Exception as error:
                                import traceback
                                Log.warn(
                                    f"[{self.connection.remote_address}]: {error}")
                    except websockets.exceptions.ConnectionClosedOK:
                        import traceback
                        print(traceback.format_exc())
                    except websockets.exceptions.ConnectionClosedError as recv_error:
                        Log.warn(f"websocket connection 'ws: // localhost: {self.port}' closed while recv: {recv_error}")
                    except Exception:
                        import traceback
                        print(traceback.format_exc())
                        self.connection = None
            except KeyboardInterrupt:
                pass
            except (ConnectionRefusedError, ConnectionResetError) as cr_error:
                if self._conn_try == 0:
                    Log.warn(f"{cr_error}")
            except Exception as error:
                import traceback
                print(traceback.format_exc())
                if self._conn_try == 0:
                    import traceback
                    print(traceback.format_exc())
            finally:
                if self._conn_try == 0:
                    Log.info(f"client disconnected")
                self._conn_try += 1
                time.sleep(1)

    def handle_callback(self, id, callback: Callable[[Any], Union[str, object]], args=[]) -> None:
        Log.info(f"handle callback {id}: {args}")
        result = None
        error = None
        reply = ''
        try:
            result = callback(*(arg for arg in args))
            if not isinstance(result, str):
                result = json.dumps(result, cls=SelfEncoder)
        except Exception as err:
            import traceback
            error = traceback.format_exc()
            error = err
        if error is None:
            reply = f'{{"id": {id}, "result": {result}}}'
        else:
            reply = f'{{"id": {id}, "error": "{error}"}}'
        self.queue.put(QueueItem(reply, priority=0))

    def publish(self, uri: str, message: str, latched=False) -> None:
        # TODO: add resend_after_connect?
        latched_value = 'true' if latched else 'false'
        msg = message
        if not isinstance(msg, str):
            msg = json.dumps(msg, cls=SelfEncoder)
        self.queue.put(
            QueueItem(f'{{"uri": "{uri}", "message": {msg}, "latched": {latched_value}}}', priority=1))

    # blocking call
    def _send_handler(self):
        try:
            while not self._shutdown:
                try:
                    item = self.queue.get()
                    if self.connection:
                        self.connection.send(item.data)
                except websockets.exceptions.ConnectionClosedOK:
                    pass
                except websockets.exceptions.ConnectionClosedError as send_error:
                    Log.warn(f"websocket connection 'ws: // localhost: {self.port}' closed: {send_error}")
                except (ConnectionRefusedError, ConnectionResetError) as send_reset:
                    Log.warn(f"websocket connection 'ws: // localhost: {self.port}' reset: {send_reset}")
                    pass
                except Exception:
                    import traceback
                    print(traceback.format_exc())
        except Exception:
            import traceback
            print(traceback.format_exc())
        finally:
            pass
