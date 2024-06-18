
import json
import os
from fkie_mas_pylib.defines import NMD_DEFAULT_PORT
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.websocket import globals

def ws_register_method(uri: str, callback):
    globals.WS_REGISTRATIONS[uri] = callback


def ws_publish_to(uri: str, message: str) -> bool:
    print(f"PUBLISH {uri}: {message} === {type(message)}")
    # global WS_CONNECTIONS
    message = message
    if not isinstance(message, str):
        message = json.dumps(message, cls=SelfEncoder)
    for con in globals.WS_CONNECTIONS:
        print(f"  add to queue")
        con.publish(uri, message)


def ws_port() -> int:
    if 'ROS_VERSION' in os.environ and os.environ['ROS_VERSION'] == "1":
        from urllib.parse import urlparse
        from fkie_mas_pylib.system import ros1_masteruri
        muri = ros1_masteruri.from_master(True)
        o = urlparse(muri)
        port = o.port
        if o.scheme == 'http':
            port += 600
        return port
    else:
        # use defaults for ROS2
        ros_domain_id = 0
        if 'ROS_DOMAIN_ID' in os.environ:
            ros_domain_id = int(os.environ['ROS_DOMAIN_ID'])
        return NMD_DEFAULT_PORT + ros_domain_id
