# The MIT License (MIT)

# Copyright (c) 2014-2024 Fraunhofer FKIE, Alexander Tiderko

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import asyncio
import json
from types import SimpleNamespace
import websockets
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.interface import SelfEncoder


class QueueItem:

    def __init__(self, data: str, priority=1):
        self.data = data
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority


class WebSocketClient:

    def __init__(self, port: int, asyncio_loop: asyncio.AbstractEventLoop):
        self.port = port
        self._shutdown = False
        self.queue = asyncio.Queue()
        self.subscriptions = {}
        self.rpcs = {}
        self.websocket = None
        self._conn_try = 0
        self._msg_id = 0
        self.asyncio_loop = asyncio_loop
        self.asyncio_loop.create_task(self.recv_handler())
        self.asyncio_loop.create_task(self._send_handler())

    def shutdown(self):
        self._shutdown = True

    def subscribe(self, uri: str, callback):
        self.subscriptions[uri] = callback
        if self.websocket:
            Log.info(f"subscribe to {uri}")
            # send subscription
            self._msg_id += 1
            request = {"uri": "sub", "id": self._msg_id, "params": [uri]}
            self.publish("sub", json.dumps(request))

    def register(self, uri: str, callback):
        self.rpcs[uri] = callback
        if self.websocket:
            Log.info(f"rigister callback for {uri}")
            # send registration
            self._msg_id += 1
            request = {"uri": "reg", "id": self._msg_id, "params": [uri]}
            self.publish("reg", json.dumps(request))

    async def recv_handler(self):
        while not self._shutdown:
            try:
                uri = f"ws://localhost:{self.port}"
                async with websockets.connect(uri) as websocket:
                    try:
                        self._conn_try = 0
                        Log.info(f"connected to {uri}")
                        self.websocket = websocket
                        # send subscriptions
                        for key in self.subscriptions.keys():
                            Log.info(f"subscribe to {key}")
                            self._msg_id += 1
                            request = {"uri": "sub",
                                       "id": self._msg_id, "params": [key]}
                            await self.websocket.send(json.dumps(request))
                        # send registrations
                        for key in self.rpcs.keys():
                            Log.info(f"register callback for {key}")
                            self._msg_id += 1
                            request = {"uri": "reg",
                                       "id": self._msg_id, "params": [key]}
                            await self.websocket.send(json.dumps(request))
                        message = await websocket.recv()
                        while message:
                            try:
                                msg = json.loads(message,
                                                 object_hook=lambda d: SimpleNamespace(**d))
                                if not hasattr(msg, 'uri') and not hasattr(msg, 'id'):
                                    Log.warn(
                                        f"[{self.websocket}]: received malformed message (without uri and id) {message}")
                                else:
                                    if hasattr(msg, 'message'):
                                        if msg.uri in self.subscriptions:
                                            self.subscriptions[msg.uri](
                                                msg.message)
                                    if hasattr(msg, 'params'):
                                        self.handle_callback(
                                            msg.id, self.rpcs[msg.uri], msg.params if hasattr(msg, 'params') else [])
                            except Exception as error:
                                import traceback
                                print(traceback.format_exc())
                                Log.warn(
                                    f"[{self.websocket.origin}]: {error}")
                            message = await websocket.recv()
                    except websockets.exceptions.ConnectionClosedOK:
                        pass
                    except websockets.exceptions.ConnectionClosedError:
                        pass
                    except Exception:
                        import traceback
                        print(traceback.format_exc())
                        self.websocket = None
            except KeyboardInterrupt:
                pass
            except (ConnectionRefusedError, ConnectionResetError) as cr_error:
                if self._conn_try == 0:
                    Log.warn(f"{cr_error}")
            except Exception as error:
                if self._conn_try == 0:
                    import traceback
                    print(traceback.format_exc())
            finally:
                if self._conn_try == 0:
                    Log.info(f"client disconnected")
                self._conn_try += 1
                await asyncio.sleep(1.0)

    def handle_callback(self, id, callback, args=[]):
        Log.debug(f"handle callback {id}: {args}")
        result = None
        error = None
        reply = ''
        try:
            result = callback(*(arg for arg in args))
            if not isinstance(result, str):
                result = json.dumps(result, cls=SelfEncoder)
        except Exception:
            import traceback
            error = traceback.format_exc()
        if error is None:
            reply = f'{{"id": {id}, "result": {result}}}'
        else:
            reply = f'{{"id": {id}, "error": {error}}}'
        self.queue.put_nowait(QueueItem(reply, priority=0))

    def publish(self, uri: str, message: str, latched=False):
        # TODO: resend_after_connect
        latched_value = 'true' if latched else 'false'
        self.queue.put_nowait(
            QueueItem(f'{{"uri": "{uri}", "message": {message}, "latched": {latched_value}}}', priority=1))

    async def _send_handler(self):
        try:
            while not self._shutdown:
                try:
                    item = await self.queue.get()
                    if self.websocket:
                        # Implement custom logic based on queue.qsize() and
                        # websocket.transport.get_write_buffer_size() here.
                        await self.websocket.send(item.data)
                except websockets.exceptions.ConnectionClosedOK:
                    pass
                except websockets.exceptions.ConnectionClosedError:
                    pass
                except (ConnectionRefusedError, ConnectionResetError):
                    pass
                except Exception:
                    import traceback
                    print(traceback.format_exc())
        except Exception:
            import traceback
            print(traceback.format_exc())
        finally:
            pass
