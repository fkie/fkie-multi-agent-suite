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


import json
import threading
import websockets
import websockets.sync
import websockets.sync.server
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.websocket.handler import WebSocketHandler


class WebSocketServer:

    def __init__(self):
        self._shutdown = False
        self.subscriptions = {}
        self.handler = set()
        self.registrations = {}
        self.rpcs = {}
        self._spin_thread = None
        self._server = None

    def shutdown(self):
        self._shutdown = True
        if (self._server):
            self._server.shutdown()
            self._server = None

    def ws_handler(self, websocket):
        handler = WebSocketHandler(self, websocket)
        self.handler.add(handler)
        handler.spin()
        self.handler.remove(handler)
        # TODO: remove all registered rpcs

    def spin(self, port=35430):
        Log.info(
            f"Open Websocket on port {port}")
        try:
            with websockets.sync.server.serve(self.ws_handler, "0.0.0.0", port) as server:
                self._server = server
                server.serve_forever()
        except Exception:
            import traceback
            print("Error while start websocket server: ", traceback.format_exc())

    def start_threaded(self, port=35430):
        self._spin_thread = threading.Thread(
            target=self.spin, args=(port,), daemon=True)
        self._spin_thread.start()

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
