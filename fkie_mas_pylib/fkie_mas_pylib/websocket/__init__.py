# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
from fkie_mas_pylib.defines import MAX_ROS1_NETWORKS, NMD_DEFAULT_PORT


def ws_port() -> int:
    '''
    Depending on the ROS version, different ports are used for the websocket server.
    The ROS_DOMAIN_ID is also added for ROS1 and ROS2.
    '''
    ros_domain_id = 0
    if 'ROS_DOMAIN_ID' in os.environ:
        ros_domain_id = int(os.environ['ROS_DOMAIN_ID'])

    if 'ROS_VERSION' in os.environ and os.environ['ROS_VERSION'] == "1":
        from urllib.parse import urlparse
        from fkie_mas_pylib.system import ros1_masteruri
        muri = ros1_masteruri.from_master(True)
        o = urlparse(muri)
        diffRosPort = o.port - 11311
        if o.scheme == 'http':
            diffRosPort *= MAX_ROS1_NETWORKS
        return NMD_DEFAULT_PORT + 255 + ros_domain_id + diffRosPort
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
