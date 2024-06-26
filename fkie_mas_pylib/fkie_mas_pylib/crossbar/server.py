import errno
import json
import os
import subprocess
import shutil
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.defines import NMD_DEFAULT_PORT
from fkie_mas_pylib.defines import SETTINGS_PATH
from fkie_mas_pylib.system.screen import test_screen
from fkie_mas_pylib.system.screen import create_screen_cfg


CROSSBAR_PATH = os.path.join(os.path.join(
    os.path.expanduser('~'), 'tmp'), '.crossbar')

CROSSBAR_BIN_PATHS = ['crossbar', '/snap/bin/crossbar']

CROSSBAR_CONFIG_JSON = {
    "version": 2,
    "controller": {},
    "workers": [
        {
            "type": "router",
            "options": {
                "pythonpath": [
                    ".."
                ]
            },
            "realms": [
                {
                    "name": "ros",
                    "roles": [
                        {
                            "name": "anonymous",
                            "permissions": [
                                {
                                    "uri": "ros.",
                                    "match": "prefix",
                                    "allow": {
                                        "call": True,
                                        "register": True,
                                        "publish": True,
                                        "subscribe": True
                                    },
                                    "disclose": {
                                        "caller": False,
                                        "publisher": False
                                    },
                                    "cache": True
                                }
                            ]
                        }
                    ],
                    "store": {
                        "type": "memory",
                        "event-history": [
                            {
                                "uri": "ros.provider.warnings",
                                "match": "exact",
                                "limit": 10
                            }
                        ]
                    }
                }
            ],
            "transports": [
                {
                    "type": "websocket",
                    "endpoint": {
                        "type": "tcp",
                        "port": 11911
                    },
                    "options": {
                        "echo_close_codereason": True
                    }
                }
            ]
        }
    ]
}


def port() -> int:
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


def crossbar_create_config(port: int) -> None:
    global CROSSBAR_PATH
    CROSSBAR_PATH = os.path.join(os.path.join(os.path.expanduser(
        '~'), os.path.join('tmp', 'crossbar_%d' % port)), '.crossbar')
    try:
        os.makedirs(CROSSBAR_PATH)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
    with open(os.path.join(CROSSBAR_PATH, 'config.json'), "w") as file:
        CROSSBAR_CONFIG_JSON["workers"][0]["transports"][0]["endpoint"]["port"] = port
        file.write(json.dumps(CROSSBAR_CONFIG_JSON, ensure_ascii=False))


def crossbar_start_server(port: int) -> str:
    crossbar_create_config(port)
    crossbar_bin = None
    for sbp in CROSSBAR_BIN_PATHS:
        crossbar_bin = shutil.which(sbp)
        if crossbar_bin is not None:
            break
    if crossbar_bin is None:
        try:
            Log.error(
                "shutil.which('crossbar'): Could not find [crossbar], please check your PATH variable.")
        except:
            import sys
            sys.stderr.write(
                "shutil.which('crossbar'): Could not find [crossbar], please check your PATH variable.")
        return ""

    test_screen()
    cfg_file = create_screen_cfg()
    cmd = [shutil.which('screen'), "-c", cfg_file, "-dmS", "_crossbar_server_%d" %
           port, crossbar_bin, "start", "--cbdir", CROSSBAR_PATH]
    print(' '.join(cmd))
    p = subprocess.Popen([shutil.which('screen'), "-c", cfg_file, "-dmS", "_crossbar_server_%d" %
                          port, crossbar_bin, "start", "--cbdir", CROSSBAR_PATH])
    return CROSSBAR_PATH
