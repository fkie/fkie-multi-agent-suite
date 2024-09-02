# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
from fkie_mas_pylib.defines import MAX_ROS1_NETWORKS, NMD_DEFAULT_PORT


def ws_port(networkId = 0) -> int:
    '''
    Depending on the ROS version, different ports are used for the websocket server.
    The ROS_DOMAIN_ID is also added for ROS2.
    '''
    if 'ROS_VERSION' in os.environ and os.environ['ROS_VERSION'] == "1":
        from urllib.parse import urlparse
        from fkie_mas_pylib.system import ros1_masteruri
        muri = ros1_masteruri.from_master(True)
        o = urlparse(muri)
        diffRosPort = o.port - 11311
        if o.scheme == 'http':
            diffRosPort *= MAX_ROS1_NETWORKS
        return NMD_DEFAULT_PORT + 255 + networkId + diffRosPort
    else:
        # use defaults for ROS2
        ros_domain_id = 0
        if 'ROS_DOMAIN_ID' in os.environ:
            ros_domain_id = int(os.environ['ROS_DOMAIN_ID'])
        return NMD_DEFAULT_PORT + ros_domain_id
