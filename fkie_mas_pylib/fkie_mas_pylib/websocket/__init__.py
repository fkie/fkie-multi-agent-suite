# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
from fkie_mas_pylib.defines import MAX_ROS1_NETWORKS
from fkie_mas_pylib.defines import NMD_DEFAULT_PORT


def ws_port() -> int:
    '''
    Depending on the ROS version, different ports are used for the websocket server.
    The ROS_DOMAIN_ID is also added for ROS1 and ROS2.
    '''
    ros_version = 2
    uri_shift_port = 0
    ros_domain_id = 0
    if 'ROS_DOMAIN_ID' in os.environ:
        try:
            ros_domain_id = int(os.environ["ROS_DOMAIN_ID"])
        except Exception:
            pass
    if 'ROS_DISTRO' in os.environ:
        ros_distro = f'{os.environ["ROS_DISTRO"]}'
        if ros_distro in ['noetic']:
            ros_version = 1
    if ros_version == 1 and 'ROS_MASTER_URI' in os.environ:
        from urllib.parse import urlparse
        from fkie_mas_pylib.system import ros1_masteruri
        muri = ros1_masteruri.from_master(True)
        o = urlparse(muri)
        if o.port >= 11311:
            uri_shift_port = o.port - 11311
            if o.scheme == 'http':
                uri_shift_port *= MAX_ROS1_NETWORKS
        return NMD_DEFAULT_PORT + 255 + ros_domain_id + uri_shift_port
    else:
        # use defaults for ROS2
        return NMD_DEFAULT_PORT + ros_domain_id


def ws_port_from(mcast_port, ros_master_uri) -> int:
    '''
    Depending on the ROS version, different ports are used for the websocket server.
    The ROS_DOMAIN_ID is also added for ROS2.
    '''
    from urllib.parse import urlparse
    from fkie_mas_pylib.system import ros1_masteruri
    from fkie_mas_pylib.system.url import get_port
    master_port = get_port(ros_master_uri)
    master_port = master_port - 11311 if master_port else 0
    network_id = mcast_port - 11511
    return NMD_DEFAULT_PORT + 255 + network_id + MAX_ROS1_NETWORKS * master_port
