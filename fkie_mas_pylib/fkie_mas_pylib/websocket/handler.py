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
import time
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


class WebSocketHandler:

    def __init__(self, server, websocket: websockets.WebSocketServerProtocol, path: str, asyncio_loop: asyncio.AbstractEventLoop):
        self.server = server
        self.websocket = websocket
        self.address = f"{self.websocket.remote_address[0]}:{self.websocket.remote_address[1]}"
        Log.info(f"{self.address}: connected")
        self.path = path
        self._shutdown = False
        self.asyncio_loop = asyncio_loop
        self.queue = asyncio.Queue()
        self.subscriptions = set()
        self.asyncio_loop.create_task(self._broadcast_handler())

    def shutdown(self):
        self._shutdown = True

    async def spin(self):
        try:
            async for message in self.websocket:
                try:
                    msg = json.loads(message,
                                     object_hook=lambda d: SimpleNamespace(**d))
                    if not hasattr(msg, 'uri'):
                        Log.warn(
                            f"[{self.address}]: received malformed message (without uri) {message}")
                        reply = {
                            "error": "malformed message, should contain uri"}
                        if hasattr(msg, 'id'):
                            reply['id'] = msg.id
                        await self.queue.put(QueueItem(json.dumps(
                            reply, cls=SelfEncoder), priority=0))
                        # await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
                        continue
                    if hasattr(msg, 'id'):
                        # handle rpc calls
                        if msg.uri == 'sub':
                            # create subscription
                            Log.info(
                                f"[{self.address}]: add subscription to '{msg.params[0]}'")
                            self.subscriptions.add(msg.params[0])
                            reply = {"id": msg.id, "result": True}
                            await self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfEncoder), priority=0))
                        elif msg.uri == 'unsub':
                            # create subscription
                            try:
                                Log.info(
                                    f"[{self.address}]: remove subscription to '{msg.params[0]}'")
                                self.subscriptions.remove(msg.params[0])
                            except:
                                pass
                            reply = {"id": msg.id, "result": True}
                            await self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfEncoder), priority=0))
                        elif msg.uri == 'reg':
                            # register a method
                            self.server.rpcs(msg.params[0], self)
                            reply = {"id": msg.id, "result": True}
                            await self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfEncoder), priority=0))
                        elif msg.uri in self.server.registrations:
                            # call local method
                            self.handle_callback(
                                msg.id, self.server.registrations[msg.uri], msg.params if hasattr(msg, 'params') else [])
                        elif msg.uri in self.server.rpcs:
                            # call rpc of a registerd by a connected client
                            Log.info(
                                f"{self.address}: handle rpc for uri {msg.uri}, params: {msg.params}")
                            self.handle_callback(
                                msg.id, self.server.rpcs[msg.uri], msg.params if hasattr(msg, 'params') else [])
                        else:
                            Log.info(
                                f"rcp not found {msg.uri}, params: {msg.params}")
                            reply = {
                                "id": msg.id, "error": f"no method for ${msg.uri} registered"}
                            await self.queue.put(QueueItem(json.dumps(
                                reply, cls=SelfEncoder), priority=0))
                    elif hasattr(msg, 'message'):
                        self.server.publish(msg.uri, msg.message)
                except Exception as error:
                    import traceback
                    Log.warn(f"[{self.address}]: {error}")
        except websockets.ConnectionClosedError as close_error:
            print(f"{self.address}: {close_error}")
        except Exception as error:
            import traceback
            print(traceback.format_exc())
        finally:
            Log.info(f"{self.address}: client removed")

    def handle_callback(self, id, callback, args=[]):
        Log.debug(f"{self.address}: handle callback {id}: {args}")
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

    def publish(self, uri: str, message: str):
        if uri in self.subscriptions:
            self.queue.put_nowait(
                QueueItem(f'{{"uri": "{uri}", "message": {message}}}', priority=1))

    async def _broadcast_handler(self):
        try:
            while not self._shutdown:
                try:
                    # TODO Implement custom logic based on queue.qsize() and
                    # websocket.transport.get_write_buffer_size() here.
                    item = await self.queue.get()
                    await self.websocket.send(item.data)
                except Exception:
                    import traceback
                    Log.info(traceback.format_exc())
        except Exception:
            import traceback
            Log.info(traceback.format_exc())
        finally:
            pass

    async def _call_rpc(self, uri: str, params=[]):
        pass
