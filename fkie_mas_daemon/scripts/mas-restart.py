#!/usr/bin/env python3

import json
import os
import psutil
import shlex
import subprocess
import time
from types import SimpleNamespace
import websockets
import websockets.sync
import websockets.sync.client

from fkie_mas_pylib.defines import SETTINGS_PATH
from fkie_mas_pylib.interface import SelfAllEncoder
from fkie_mas_pylib.websocket import ws_port


if __name__ == '__main__':

    launch_files = []

    # get loaded launch files
    ros_domain_id = 0
    if "ROS_DOMAIN_ID" in os.environ:
        ros_domain_id = os.environ["ROS_DOMAIN_ID"]
    print(f"connect to daemon on ROS_DOMAIN_ID: {ros_domain_id}")
    print(f"connect to: ws://localhost:{ws_port()}")
    with websockets.sync.client.connect(f"ws://localhost:{ws_port()}", max_size=2**22) as connection:
        print("request launch files using uri 'ros.launch.get_list'")
        connection.send('{"uri": "ros.launch.get_list", "id": -1}')
        reply = connection.recv(3)
        print("reply received")
        msg = json.loads(reply, object_hook=lambda d: SimpleNamespace(**d))
        has_id = hasattr(msg, 'id')
        if has_id and msg.id == -1:
            print("store loaded launch files")
            launch_files = msg.result
            for launch in launch_files:
                print(f"path: {launch.path}")
                print(f"  args: {launch.args}")
        else:
            print("  but the reply is not for me")

    # stop running mas nodes
    cmd_daemon_str = ""
    cmd_discovery_str = ""
    mas_processes = []
    try:
        for ps_it in psutil.process_iter():
            try:
                cmd_str = ' '.join(ps_it.cmdline())
                if cmd_str.find(SETTINGS_PATH) > -1:
                    # is it mas-daemon?
                    if (cmd_str.find('mas-daemon') > 0):
                        print(f"stop mas daemon with pid: {ps_it.pid}")
                        cmd_daemon_str = cmd_str
                        mas_processes.append(ps_it)
                        ps_it.terminate()
                    # is it mas-discovery?
                    if (cmd_str.find('mas-discovery') > 0):
                        print(f"stop mas discovery with pid: {ps_it.pid}")
                        cmd_discovery_str = cmd_str
                        mas_processes.append(ps_it)
                        ps_it.terminate()
            except (psutil.ZombieProcess, psutil.NoSuchProcess):
                # ignore errors because of zombie processes or non-existent (terminated child?) processes
                pass
            except Exception as error:
                import traceback
                print(traceback.format_exc())
        gone, alive = psutil.wait_procs(mas_processes, timeout=3)
        for p in alive:
            p.kill()
        result = True
    except Exception as error:
        import traceback
        print(traceback.format_exc())

    # start nodes
    if cmd_daemon_str:
        cmd_daemon_str = cmd_daemon_str.replace("SCREEN", "screen")
        print(f"start daemon: {cmd_daemon_str}")
        subprocess.Popen(shlex.split(cmd_daemon_str))
    if cmd_discovery_str:
        cmd_discovery_str = cmd_discovery_str.replace("SCREEN", "screen")
        print(f"start discovery: {cmd_discovery_str}")
        subprocess.Popen(shlex.split(cmd_discovery_str))

    # load launch files
    def load_launch_files(launch_files):
        # load stored launch files
        print(f"connect to: ws://localhost:{ws_port()}")
        with websockets.sync.client.connect(f"ws://localhost:{ws_port()}", max_size=2**22) as connection:
            for launch in launch_files:
                message = {
                    "uri": "ros.launch.load",
                    "id": 0,
                    "params": [
                        {
                            "path": launch.path,
                            "args": launch.args,
                            "ros_package": "",
                            "launch": "",
                            "host": "",
                            "request_args": [],
                        }
                    ]
                }
                message_str = json.dumps(message, cls=SelfAllEncoder)
                print(f"MESSAGE: {message_str}")
                connection.send(message_str)
                reply = connection.recv(3)
                print(f"REPLY: {reply}")

    tries = 0
    while tries < 10:
        try:
            load_launch_files(launch_files)
            tries = 10
            print(f"load successfully")
        except (EOFError, ConnectionRefusedError) as e:
            print(f"... retry")
            time.sleep(1)
            tries += 1