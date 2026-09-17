# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import re

from fkie_mas_pylib.system.host import ros_host_suffix

if os.environ["ROS_VERSION"] == "1":
    SCREEN_SLASH_SEP = "_"
    """this character is used to replace the slashes in ROS-Names for ROS1 nodes."""
    RESPAWN_SCRIPT = "rosrun fkie_mas_daemon mas-respawn"
    """:var RESPAWN_SCRIPT: start prefix to launch ROS-Nodes with respawn script"""
else:
    SCREEN_SLASH_SEP = "."
    """this character is used to replace the slashes in ROS-Names for ROS2 nodes."""
    RESPAWN_SCRIPT = "ros2 run fkie_mas_daemon mas-respawn"
    """:var RESPAWN_SCRIPT: start prefix to launch ROS-Nodes with respawn script"""

SEP = "/"
PRIV_NAME = "~"
NM_NAMESPACE = "/mas"
# ros_distro = f"_{os.environ['ROS_DISTRO']}" if 'ROS_DISTRO' in os.environ else ''
nm_name_suffix = ros_host_suffix()
if nm_name_suffix:
    nm_name_suffix = f"_{nm_name_suffix}"
ROS_DOMAIN_ID = os.environ["ROS_DOMAIN_ID"] if "ROS_DOMAIN_ID" in os.environ else "0"
NM_DISCOVERY_NAME = f"_discovery_{ROS_DOMAIN_ID}{nm_name_suffix}"
NM_DAEMON_NAME = f"_daemon_{ROS_DOMAIN_ID}{nm_name_suffix}"
NM_PUBLISHER_NAME = f"_publisher_{ROS_DOMAIN_ID}{nm_name_suffix}"
NM_SUBSCRIBER_NAME = f"_subscriber_{ROS_DOMAIN_ID}{nm_name_suffix}"
NM_ACTION_NAME = f"_action_{ROS_DOMAIN_ID}{nm_name_suffix}"
NM_ACTION_INTROSPECTION_NAME = f"_action_introspection_{ROS_DOMAIN_ID}{nm_name_suffix}"
EMPTY_PATTERN = re.compile("\b", re.I)
SEARCH_IN_EXT = [".launch", ".yaml", ".conf", ".cfg", ".py", ".iface", ".nmprofile", ".sync", ".test", ".xml", ".xacro"]

PACKAGE_FILE = "package.xml"

try:
    import rospkg

    LOG_PATH = rospkg.get_log_dir()
except ImportError:
    LOG_PATH = (
        "".join([os.environ.get("ROS_LOG_DIR"), os.path.sep])
        if os.environ.get("ROS_LOG_DIR")
        else os.path.join(os.path.expanduser("~"), ".ros/log/")
    )
""":var LOG_PATH: logging path where all screen configuration and log files are stored."""

SETTINGS_PATH = os.path.expanduser("~/.config/ros.fkie/")

SCREEN = "/usr/bin/screen"
""":var SCREEN: Defines the path to screen binary."""

SCREEN_NAME_MAX_CHARS = 74

MAX_ROS1_NETWORKS = 101

NMD_DEFAULT_PORT = 35430
""":var NMD_DEFAULT_PORT: default port of node manager daemon."""


# def ros1_subscriber_nodename_tuple(topic: str) -> Tuple[str, str]:  # namespace, name
#     return ('', '')


def ros2_publisher_nodename_tuple(topic: str) -> tuple[str, str]:  # namespace, name
    namespace = os.path.join(NM_NAMESPACE, NM_PUBLISHER_NAME)
    topic_parts = topic.strip("/").split("/")
    node_name = topic_parts[-1]
    topic_ns = "/".join(topic_parts[:-1])
    if topic_ns:
        namespace = os.path.join(namespace, topic_ns)
    return (namespace, node_name)


def ros2_subscriber_nodename_tuple(topic: str) -> tuple[str, str]:  # namespace, name
    namespace = os.path.join(NM_NAMESPACE, NM_SUBSCRIBER_NAME)
    topic_parts = topic.strip("/").split("/")
    node_name = topic_parts[-1]
    topic_ns = "/".join(topic_parts[:-1])
    if topic_ns:
        namespace = os.path.join(namespace, topic_ns)
    return (namespace, node_name)


def ros2_action_nodename_tuple(action_name: str):
    """Returns (namespace, name) tuple for an action client node."""
    namespace = os.path.join(NM_NAMESPACE, NM_ACTION_NAME)
    action_parts = action_name.replace(".", "/").strip("/").split("/")
    node_name = action_parts[-1]
    action_ns = "/".join(action_parts[:-1])
    if action_ns:
        namespace = os.path.join(namespace, action_ns)
    return (namespace, node_name)


def ros2_action_introspection_nodename_tuple(action_name: str):
    """Returns (namespace, name) tuple for an action client node."""
    namespace = os.path.join(NM_NAMESPACE, NM_ACTION_INTROSPECTION_NAME)
    action_parts = action_name.replace(".", "/").strip("/").split("/")
    node_name = action_parts[-1]
    action_ns = "/".join(action_parts[:-1])
    if action_ns:
        namespace = os.path.join(namespace, action_ns)
    return (namespace, node_name)


def ros2_service_introspection_nodename_tuple(service_name: str):
    namespace = os.path.join(NM_NAMESPACE, f"_service_introspection_{ROS_DOMAIN_ID}{nm_name_suffix}")
    service_parts = service_name.replace(".", "/").strip("/").split("/")
    node_name = service_parts[-1]
    service_ns = "/".join(service_parts[:-1])
    if service_ns:
        namespace = os.path.join(namespace, service_ns)
    return (namespace, node_name)
