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
import threading
from types import SimpleNamespace
import websockets
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.interface import SelfEncoder
# from fkie_mas_pylib.websocket.globals import WS_CONNECTIONS, WS_REGISTRATIONS
from fkie_mas_pylib.websocket import globals
from fkie_mas_pylib.websocket import ws_publish_to


class QueueItem:

    def __init__(self, data: str, priority=1):
        self.data = data
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority


class WebSocketHandler:

    def __init__(self, websocket: websockets.WebSocketServerProtocol, path: str, asyncio_loop: asyncio.AbstractEventLoop):
        print(f"CREATE {websocket.origin}")
        self.websocket = websocket
        self.path = path
        self._shutdown = False
        self.asyncio_loop = asyncio_loop
        self.queue = asyncio.Queue()
        globals.WS_CONNECTIONS.add(self)
        self.subscriptions = set()

        self.asyncio_loop.create_task(self._broadcast_handler())

    def shutdown(self):
        self._shutdown = True

    async def spin(self):
        try:
            print(f"handler started {self.websocket}")
            async for message in self.websocket:
                try:
                    print(message)
                    msg = json.loads(message,
                                     object_hook=lambda d: SimpleNamespace(**d))
                    print(f"message parsed: {msg}")
                    if not hasattr(msg, 'uri'):
                        Log.warn(
                            f"[{self.websocket}]: received malformed message (without uri) {message}")
                        reply = {
                            "error": "malformed message, should contain uri"}
                        if hasattr(msg, 'id'):
                            reply['id'] = msg.id
                        self.queue.put_nowait(QueueItem(json.dumps(
                            reply, cls=SelfEncoder), priority=0))
                        # await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
                        continue
                    if hasattr(msg, 'id'):
                        # handle rpc calls
                        if msg.uri == 'sub':
                            # create subscription
                            Log.info(
                                f"[{self.websocket.origin}]: add subscription to '{msg.params[0]}'")
                            self.subscriptions.add(msg.params[0])
                            reply = {"id": msg.id, "result": True}
                            self.queue.put_nowait(QueueItem(json.dumps(
                                reply, cls=SelfEncoder), priority=0))
                            # await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
                        elif msg.uri == 'unsub':
                            # create subscription
                            try:
                                self.subscriptions.remove(msg.params[0])
                            except:
                                pass
                            reply = {"id": msg.id, "result": True}
                            self.queue.put_nowait(QueueItem(json.dumps(
                                reply, cls=SelfEncoder), priority=0))
                            # await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
                        elif msg.uri in globals.WS_REGISTRATIONS:
                            print(
                                f"handle uri {msg.uri}, params: {msg.params}")
                            # task = asyncio.create_task(
                            #     self.handle_callback(msg.id, WS_REGISTRATIONS[msg.uri], msg.params if hasattr(msg, 'params') else []))
                            # await asyncio.gather(task)
                            self.handle_callback(
                                msg.id, globals.WS_REGISTRATIONS[msg.uri], msg.params if hasattr(msg, 'params') else [])
                            # asyncio.run_coroutine_threadsafe(
                            #     self.handle_callback(msg.id, WS_REGISTRATIONS[msg.uri], msg.params if hasattr(msg, 'params') else []), self.asyncio_loop)
                        else:
                            print(
                                f"not found {msg.uri}, params: {msg.params}")
                            reply = {
                                "id": msg.id, "error": f"no method for ${msg.uri} registered"}
                            self.queue.put_nowait(QueueItem(json.dumps(
                                reply, cls=SelfEncoder), priority=0))
                            # await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
                    elif hasattr(msg, 'message'):
                        print(f"publish to {msg.uri}: {msg.message}")
                        ws_publish_to(msg.uri, msg.message)
                    # await asyncio.sleep(0)
                except Exception as error:
                    import traceback
                    print(traceback.format_exc())
                    Log.warn(
                        f"[{self.websocket.origin}]: {error}")
        except websockets.ConnectionClosedError:
            import traceback
            print(traceback.format_exc())
        except Exception as error:
            import traceback
            print(traceback.format_exc())
        finally:
            print("handler removed")
            # global WS_CONNECTIONS
            globals.WS_CONNECTIONS.remove(self)
            # self._broadcast_task.cancel()

    def handle_callback(self, id, callback, args=[]):
        print(f"handle callback {id}: {args}")
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
            print(f"ERRORRR {error}")
        if error is None:
            reply = f'{{"id": {id}, "result": {result}}}'
        else:
            reply = f'{{"id": {id}, "error": {error}}}'
            print(f"  error: ${error}")
        self.queue.put_nowait(QueueItem(reply, priority=0))
        # await self.websocket.send(reply)

    def publish(self, uri: str, message: str):
        if uri in self.subscriptions:
            print(f"  {uri} add")
            self.queue.put_nowait(QueueItem(f'{{"uri": "{uri}", "message": {message}}}', priority=1))

    async def _broadcast_handler(self):
        print(f"***** broadcast START")
        try:
            while not self._shutdown:
                try:
                    print(f"***** broadcast get")
                    # TODO Implement custom logic based on queue.qsize() and
                    # websocket.transport.get_write_buffer_size() here.
                    item = await self.queue.get()
                    await self.websocket.send(item.data)
                except Exception:
                    import traceback
                    print(traceback.format_exc())
                    print(f"DATA: {item.data}")
        except Exception:
            import traceback
            print(traceback.format_exc())
        finally:
            print(f"*****END broadcast")
