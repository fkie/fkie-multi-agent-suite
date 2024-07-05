# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os

import roslib
import rospy
from urllib.parse import urlparse
import xmlrpc.client as xmlrpcclient


MASTERURI = None


def from_ros() -> str:
    '''
    Returns the master URI depending on ROS distribution API.

    :return: ROS master URI
    :rtype: str
    :see: rosgraph.rosenv.get_master_uri() (fuerte)
    :see: roslib.rosenv.get_master_uri() (prior)
    '''
    try:
        import rospkg.distro
        distro = rospkg.distro.current_distro_codename()
        if distro in ['electric', 'diamondback', 'cturtle']:
            return roslib.rosenv.get_master_uri()
        else:
            import rosgraph
            return rosgraph.rosenv.get_master_uri()
    except Exception:
        return os.environ['ROS_MASTER_URI']


def from_master(from_env_on_error:bool = False) -> str:
    '''
    Requests the ROS master URI from the ROS master through the RPC interface and
    returns it. The 'materuri' attribute will be set to the requested value.

    :return: ROS master URI
    :rtype: str or None
    '''
    global MASTERURI
    result = MASTERURI
    try:
        if MASTERURI is None:
            masteruri = from_ros()
            result = masteruri
            master = xmlrpcclient.ServerProxy(masteruri)
            code, _, MASTERURI = master.getUri(rospy.get_name())
            if code == 1:
                result = MASTERURI
    except Exception as err:
        if from_env_on_error:
            result = from_ros()
        else:
            raise err
    return result


def get_ros_home():
    '''
    Returns the ROS HOME depending on ROS distribution API.

    :return: ROS HOME path
    :rtype: str
    '''
    try:
        import rospkg.distro
        distro = rospkg.distro.current_distro_codename()
        if distro in ['electric', 'diamondback', 'cturtle']:
            import roslib.rosenv
            return roslib.rosenv.get_ros_home()
        else:
            from rospkg import get_ros_home
            return get_ros_home()
    except Exception:
        from roslib import rosenv
        return rosenv.get_ros_home()
