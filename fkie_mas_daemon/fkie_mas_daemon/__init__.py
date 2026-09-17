# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from rclpy.node import Node

from .action_client_node import RosActionClientLauncher
from .action_introspection import RosActionIntrospectionLauncher
from .ros_node import RosNodeLauncher
from .service_introspection_node import RosServiceIntrospectionLauncher
from .subscriber_node import RosSubscriberLauncher

# from pkg_resources import get_distribution, DistributionNotFound
# try:
#     __version__ = get_distribution(__name__).version
# except DistributionNotFound:
#     # package is not installed
#     pass

# the ros_node is assigned in :class:RosNodeLauncher while init
ros_node: Node = None
launcher: RosNodeLauncher = None


def main():
    global launcher
    launcher = RosNodeLauncher()
    launcher.spin()


def subscriber():
    global launcher
    launcher = RosSubscriberLauncher()
    launcher.spin()


def action_client():
    global launcher
    launcher = RosActionClientLauncher()
    launcher.spin()


def action_introspection():
    global launcher
    launcher = RosActionIntrospectionLauncher()
    launcher.spin()


def service_introspection():
    global launcher
    launcher = RosServiceIntrospectionLauncher()
    launcher.spin()
