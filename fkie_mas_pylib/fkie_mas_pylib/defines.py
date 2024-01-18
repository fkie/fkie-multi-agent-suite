
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

import os
import re
from typing import Tuple
from fkie_mas_pylib.system.host import ros_host_suffix


if os.environ['ROS_VERSION'] == "1":
    SCREEN_SLASH_SEP = '_'
    '''this character is used to replace the slashes in ROS-Names for ROS1 nodes.'''
    RESPAWN_SCRIPT = 'rosrun fkie_mas_daemon mas-respawn'
    ''':var RESPAWN_SCRIPT: start prefix to launch ROS-Nodes with respawn script'''
else:
    SCREEN_SLASH_SEP = '.'
    '''this character is used to replace the slashes in ROS-Names for ROS2 nodes.'''
    RESPAWN_SCRIPT = 'ros2 run fkie_mas_daemon mas-respawn'
    ''':var RESPAWN_SCRIPT: start prefix to launch ROS-Nodes with respawn script'''

SEP = '/'
PRIV_NAME = '~'
NM_NAMESPACE = '/mas'
#ros_distro = f"_{os.environ['ROS_DISTRO']}" if 'ROS_DISTRO' in os.environ else ''
nm_name_suffix = ros_host_suffix()
if nm_name_suffix:
    nm_name_suffix = f"_{nm_name_suffix}"
NM_DISCOVERY_NAME = f'_discovery{nm_name_suffix}'
NM_DAEMON_NAME = f'_daemon{nm_name_suffix}'
NM_SUBSCRIBER_NAME = f'_subscriber{nm_name_suffix}'
EMPTY_PATTERN = re.compile('\b', re.I)
SEARCH_IN_EXT = ['.launch', '.yaml', '.conf', '.cfg',
                 '.iface', '.nmprofile', '.sync', '.test', '.xml', '.xacro']

PACKAGE_FILE = 'package.xml'

try:
    import rospkg
    LOG_PATH = rospkg.get_log_dir()
except ImportError:
    LOG_PATH = ''.join([os.environ.get('ROS_LOG_DIR'), os.path.sep]) if os.environ.get(
        'ROS_LOG_DIR') else os.path.join(os.path.expanduser('~'), '.ros/log/')
''':var LOG_PATH: logging path where all screen configuration and log files are stored.'''

SETTINGS_PATH = os.path.expanduser('~/.config/ros.fkie/')

SCREEN = "/usr/bin/screen"
''':var SCREEN: Defines the path to screen binary.'''

SCREEN_NAME_MAX_CHARS = 74

GRPC_TIMEOUT = 15.0
''':var GRPC_TIMEOUT: timeout for connection to remote gRPC-server'''

GRPC_SERVER_PORT_OFFSET = 1010
''':var GRPC_SERVER_PORT_OFFSET: offset to the ROS-Master port.'''

NMD_DEFAULT_PORT = 11811
''':var NMD_DEFAULT_PORT: default port of node manager daemon.'''


# def ros1_subscriber_nodename_tuple(topic: str) -> Tuple[str, str]:  # namespace, name
#     return ('', '')

def ros2_subscriber_nodename_tuple(topic: str) -> Tuple[str, str]:  # namespace, name
    namespace = os.path.join(NM_NAMESPACE, NM_SUBSCRIBER_NAME)
    topic_parts = topic.strip('/').split('/')
    node_name = topic_parts[-1]
    topic_ns = '/'.join(topic_parts[:-1])
    if topic_ns:
        namespace = os.path.join(namespace, topic_ns)
    return (namespace, node_name)
