
import json
from fkie_mas_pylib.crossbar.base_session import SelfEncoder
from.globals import WS_CONNECTIONS, WS_REGISTRATIONS

def ws_register_method(uri: str, callback):
    global WS_REGISTRATIONS
    WS_REGISTRATIONS[uri] = callback


def ws_publish_to(uri: str, message: str) -> bool:
    print(f"PUBLISH {uri}: {message} === {type(message)}")
    global WS_CONNECTIONS
    message = message
    if not isinstance(message, str):
        message = json.dumps(message, cls=SelfEncoder)
    for con in WS_CONNECTIONS.copy():
        print(f"  add to queue")
        con.publish(uri, message)