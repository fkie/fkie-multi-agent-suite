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
from fkie_mas_pylib.websocket import ws_publish_to


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
        # self.queue = PQueue(100)
        self.queue = asyncio.Queue()
        self.subscriptions = {}
        self.websocket = None
        self.asyncio_loop = asyncio_loop
        # self._recv_thread = threading.Thread(
        #     target=self.spin, daemon=True)
        # self._recv_thread.start()
        # self._send_thread = threading.Thread(
        #     target=self._broadcast_handler, daemon=True)
        # self._send_thread.start()
        # self._broadcast_task = asyncio.create_task(self._broadcast_handler())
        # self._broadcast_task = asyncio.run_coroutine_threadsafe(
        # self._recv_routine_task = asyncio.run_coroutine_threadsafe(
        #     self.spin(), asyncio.get_event_loop())
        # self._send_routine_task = asyncio.run_coroutine_threadsafe(
        #     self._send_handler(), asyncio.get_event_loop())
        # asyncio.run_coroutine_threadsafe(
        #     self.handle(), asyncio_loop)
        print("1")
        self.asyncio_loop.create_task(self.spin())
        print("2")
        self.asyncio_loop.create_task(self._send_handler())
        print("3")

    def shutdown(self):
        self._shutdown = True

    def subscribe(self, uri: str, callback):
        self.subscriptions[uri] = callback
        if self.websocket:
            print(f"subscribe to {uri}")
            # send subscription
            request = {"uri": "sub", "params": uri}
            self.publish("sub", json.dumps([uri]))

    async def spin(self):
        try:
            print("handler started")
            async for websocket in websockets.connect(f"ws://localhost:{self.port}"):
                try:
                    print("connected")
                    self.websocket = websocket
                    # send subscriptions
                    for key in self.subscriptions.keys():
                        print(f"subscribe to {key} on connect")
                        request = {"uri": "sub", "params": [key]}
                        await self.websocket.send(json.dumps(request))
                    async for message in websocket:
                        try:
                            print(message)
                            msg = json.loads(message,
                                             object_hook=lambda d: SimpleNamespace(**d))
                            print(f"message parsed: {msg}")
                            if not hasattr(msg, 'uri'):
                                Log.warn(
                                    f"[{self.websocket}]: received malformed message (without uri) {message}")
                                continue
                            if hasattr(msg, 'message'):
                                if msg.uri in self.subscriptions:
                                    self.subscriptions[msg.uri](msg.message)
                        except Exception as error:
                            import traceback
                            print(traceback.format_exc())
                            Log.warn(
                                f"[{self.websocket.origin}]: {error}")
                        await asyncio.sleep(0)
                except Exception:
                    import traceback
                    print(traceback.format_exc())
                    self.websocket = None
                    if self._shutdown:
                        return
                    continue
        except Exception as error:
            import traceback
            print(traceback.format_exc())
        finally:
            print("websocket client stopped")
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
            print(f"  reply: {reply}")
        else:
            reply = f'{{"id": {id}, "error": {error}}}'
            print(f"  error: ${error}")
        self.queue.put_nowait(QueueItem(reply, priority=0))
        # await self.websocket.send(reply)

    def publish(self, uri: str, message: str, resend_after_connect=False):
        #TODO: resend_after_connect
        print(f"  {uri} add")
        self.queue.put_nowait(
            QueueItem(f'{{"uri": "{uri}", "message": {message}}}', priority=1))

    # async def _broadcast_task(self) -> None:
    #     task = asyncio.create_task(self._broadcast_handler())
    #     await asyncio.gather(task)

    async def _send_handler(self):
        print(f"***** send START")
        try:
            while not self._shutdown:
                try:
                    item = await self.queue.get()
                    if self.websocket:
                        print(f"***** broadcast get")
                        # Implement custom logic based on queue.qsize() and
                        # websocket.transport.get_write_buffer_size() here.
                        print(f"PUBLISH message: {item.data}")
                        await self.websocket.send(item.data)
                        await asyncio.sleep(0)
                    else:
                        await asyncio.sleep(1)
                except Exception:
                    import traceback
                    print(traceback.format_exc())
                    print(f"DATA: {item.data}")
        except Exception:
            import traceback
            print(traceback.format_exc())
        finally:
            print(f"*****END send")
