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
from fkie_mas_pylib.crossbar.base_session import SelfEncoder


WS_REGISTRATIONS = {}


def register_ws_method(uri: str, callback):
    global WS_REGISTRATIONS
    WS_REGISTRATIONS[uri] = callback


class WebSocketHandler:
    CONNECTIONS = set()

    def __init__(self, websocket: websockets.WebSocketServerProtocol, path, async_loop):
        self.websocket = websocket
        self.path = path
        WebSocketHandler.CONNECTIONS.add(self)
        self.subscriptions = set()
        self.async_loop = async_loop
        # asyncio.run_coroutine_threadsafe(
        #     self.handle(), async_loop)

    async def spin(self):
        try:
            print("handler started")
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
                        await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
                        continue
                    if hasattr(msg, 'id'):
                        # handle rpc calls
                        if msg.uri == 'sub':
                            # create subscription
                            self.subscriptions.add(msg.params[0])
                            reply = {"id": msg.id, "result": True}
                            await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
                        if msg.uri == 'unsub':
                            # create subscription
                            try:
                                self.subscriptions.remove(msg.params[0])
                            except:
                                pass
                            reply = {"id": msg.id, "result": True}
                            await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
                        if msg.uri in WS_REGISTRATIONS:
                            print(
                                 f"handle uri {msg.uri}, params: {msg.params}")
                            # task = asyncio.create_task(
                            #     self.handle_callback(msg.id, WS_REGISTRATIONS[msg.uri], msg.params if hasattr(msg, 'params') else []))
                            # await asyncio.gather(task)
                            await self.handle_callback(msg.id, WS_REGISTRATIONS[msg.uri], msg.params if hasattr(msg, 'params') else [])
                            # asyncio.run_coroutine_threadsafe(
                            #     self.handle_callback(msg.id, WS_REGISTRATIONS[msg.uri], msg.params if hasattr(msg, 'params') else []), self.async_loop)
                        else:
                            print(
                                f"not found {msg.uri}, params: {msg.params}")
                            reply = {
                                "id": msg.id, "error": f"no method for ${msg.uri} registered"}
                            await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
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
            WebSocketHandler.CONNECTIONS.remove(self)

    async def handle_callback(self, id, callback, args=[]):
        print(f"handle callback {id}: {args}")
        result = None
        error = None
        try:
            result = callback(*(arg for arg in args))
        except Exception:
            import traceback
            error = traceback.format_exc()
            print(f"ERRORRR {error}")
        if error is None:
            reply = {"id": id, "result": result}
            print(f"  result: ${result}")
        else:
            reply = {"id": id, "error": error}
            print(f"  error: ${error}")
        await self.websocket.send(json.dumps(reply, cls=SelfEncoder))
