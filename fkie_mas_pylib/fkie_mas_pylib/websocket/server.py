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
import rclpy
import websockets
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.websocket.handler import WebSocketHandler


class QueueItem:

    def __init__(self, data: str, priority=1):
        self.data = data
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority


class WebSocketServer:

    def __init__(self, asyncio_loop: asyncio.AbstractEventLoop):
        self._shutdown = False
        self.queue = asyncio.Queue()
        self.subscriptions = {}
        self.handler = set()
        self.registrations = {}
        self.rpcs = {}
        self.asyncio_loop = asyncio_loop

    def shutdown(self):
        self._shutdown = True

    async def ws_handler(self, websocket, path):
        handler = WebSocketHandler(self, websocket, path, self.asyncio_loop)
        self.handler.add(handler)
        await handler.spin()
        self.handler.remove(handler)
        # TODO: remove all registered rpcs

    async def spin(self, port=35430):
        Log.info(
            f"Open Websocket on port {port}")
        try:
            async with websockets.serve(self.ws_handler, "0.0.0.0", port):
                # await asyncio.Future()
                while rclpy.ok():
                    await asyncio.sleep(1)
        except:
            import traceback
            print("CATCHED: ", traceback.format_exc())

    def start(self, port=35430):
        self.asyncio_loop.create_task(self.spin(port))

    def subscribe(self, uri: str, callback):
        self.subscriptions[uri] = callback

    def register(self, uri: str, callback):
        self.registrations[uri] = callback

    def publish(self, uri: str, message: str) -> bool:
        message = message
        if not isinstance(message, str):
            message = json.dumps(message, cls=SelfEncoder)
        for con in self.handler:
            con.publish(uri, message)
