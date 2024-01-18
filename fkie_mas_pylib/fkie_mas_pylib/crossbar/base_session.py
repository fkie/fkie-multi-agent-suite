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

from typing import Any
from typing import Callable
from typing import Union

import time

import json

# crossbar-io dependencies
import asyncio
from autobahn.wamp.types import ComponentConfig
from autobahn.asyncio.wamp import ApplicationSession, ApplicationRunner
from autobahn.wamp.exception import TransportLost
from asyncio import coroutine


from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system.host import get_host_name
from .server import crossbar_start_server, CROSSBAR_PATH


class SelfEncoder(json.JSONEncoder):
    def default(self, obj):
        result = {}
        for key, value in vars(obj).items():
            if key[0] != '_':
                result[key] = value
        return result


class CrossbarBaseSession(ApplicationSession):

    CROSSBAR_SERVER_STARTED = False

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911, *, test_env=False) -> None:
        self.port = port
        self.crossbar_loop = loop
        self._on_shutdown = False
        self._crossbar_subscriptions = []  # list of tuples (topic, handler)
        self._crossbar_failed_publications = {}  # (topic, message)
        self.crossbar_connected = False
        self.crossbar_connecting = False
        self.crossbar_registered = False
        self.uri = f"ws://localhost:{self.port}/ws"
        if test_env:
            return
        ApplicationSession.__init__(self, ComponentConfig(realm, {}))
        if not hasattr(self, 'realm'):
            self.realm = realm
        self.crossbar_runner = ApplicationRunner(self.uri, self.config.realm)
        self.uri = self.uri.replace('localhost', get_host_name())
        task = asyncio.run_coroutine_threadsafe(
            self.crossbar_connect(), self.crossbar_loop)

    '''
    Subscribes to a crossbar topic in async way.
    Subscribed topics in this way are subscribed after reconnection.
    '''

    def subscribe_to(self, topic: str, handler: Callable):
        self._crossbar_subscriptions.append((topic, handler))
        if self.crossbar_connected:
            asyncio.run_coroutine_threadsafe(
                self.subcribe_async(topic, handler), self.crossbar_loop)

    async def subcribe_async(self, topic: str, handler: Callable):
        try:
            await self.subscribe(handler, topic)
            Log.info(
                f"{self.__class__.__name__}: subscribed to crossbar topic '{topic}'")
        except Exception as e:
            Log.warn(
                f"{self.__class__.__name__}: could not subscribe to '{topic}': {0}".format(e))

    '''
    Publishes message to given topic without throw an exception on connection problems.
    Last failed message will be send on connect.
    '''

    def publish_to(self, topic: str, msg: Union[str, Any], resend_after_connect=True):
        encoded_msg = msg
        if not isinstance(msg, str):
            encoded_msg = json.dumps(msg, cls=SelfEncoder)
        try:
            if self.crossbar_connected:
                Log.debug(f"Publish '{topic}': {encoded_msg}")
                self.publish(topic, encoded_msg)
            elif (resend_after_connect):
                Log.info(f"publish after connect {topic}: {encoded_msg}")
                self._crossbar_failed_publications[topic] = encoded_msg  
        except TransportLost as e:
            if (resend_after_connect):
                Log.info(f"add {topic}: {encoded_msg}")
                self._crossbar_failed_publications[topic] = encoded_msg
        except Exception:
            import traceback
            Log.warn(
                f"Error while publish to {topic}: {traceback.format_exc()}")

    def shutdown(self):
        self._on_shutdown = True
        self.disconnect()

    def onConnect(self):
        Log.info(f"{self.__class__.__name__}: autobahn connected")
        self.join(self.config.realm)

    def onDisconnect(self):
        Log.info(f"{self.__class__.__name__}: autobahn disconnected")
        self.crossbar_connected = False
        self.crossbar_connecting = False
        self.crossbar_registered = False
        if not self._on_shutdown:
            self.crossbar_reconnect()

    def onLeave(self, details):
        ApplicationSession.onLeave(self, details)
        Log.debug(f"{self.__class__.__name__}.onLeave: {details}")

    @coroutine
    def onJoin(self, details):
        res = yield from self.register(self)
        if self._registrations:
            Log.info(
                f"{self.__class__.__name__}: {len(self._registrations)} crossbar procedures registered!")
            Log.info(f"{self.__class__.__name__}: list of registered uri's:")
            for _session_id, reg in self._registrations.items():
                Log.info(f"{self.__class__.__name__}:   {reg.procedure}")
        self.crossbar_registered = True
        for (topic, handler) in self._crossbar_subscriptions:
            asyncio.run_coroutine_threadsafe(
                self.subcribe_async(topic, handler), self.crossbar_loop)
        for topic, msg in self._crossbar_failed_publications.items():
            self.publish_to(topic, msg)
        self._crossbar_failed_publications.clear()

    async def crossbar_connect_async(self):
        self.crossbar_connected = False
        while not self.crossbar_connected:
            try:
                # try to connect to the crossbar server
                self.crossbar_connecting = True
                coro = await self.crossbar_runner.run(self, start_loop=False)
                (self.__crossbar_transport, self.__crossbar_protocol) = coro
                self.crossbar_connected = True
                self.crossbar_connecting = False
            except Exception as err:
                Log.debug(f"{self.__class__.__name__}: {err}")

                # try to start the crossbar server
                if not CrossbarBaseSession.CROSSBAR_SERVER_STARTED:
                    CrossbarBaseSession.CROSSBAR_SERVER_STARTED = True
                    try:
                        config_path = crossbar_start_server(self.port)
                        Log.info(
                            f"{self.__class__.__name__}: start crossbar server @ {self.uri} realm: {self.config.realm}, config: {config_path}")
                    except:
                        import traceback
                        Log.debug(f"{self.__class__.__name__}: {traceback.format_exc()}")

                self.crossbar_connecting = False
                self.crossbar_connected = False
                time.sleep(2.0)

    async def crossbar_connect(self) -> None:
        current_task = asyncio.current_task()
        if not self.crossbar_connecting:
            task = asyncio.create_task(self.crossbar_connect_async())
        else:
            task = current_task
        await asyncio.gather(task)

    def crossbar_reconnect(self):
        Log.info(f"reconnect to crossbar @ {self.uri}")
        asyncio.run_coroutine_threadsafe(
            self.crossbar_connect(), self.crossbar_loop)
