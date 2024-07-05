# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
from fkie_mas_pylib.defines import NMD_DEFAULT_PORT


def ws_port() -> int:
    if 'ROS_VERSION' in os.environ and os.environ['ROS_VERSION'] == "1":
        from urllib.parse import urlparse
        from fkie_mas_pylib.system import ros1_masteruri
        muri = ros1_masteruri.from_master(True)
        o = urlparse(muri)
        port = o.port
        if o.scheme == 'http':
            port += 24374
        return port
    else:
        # use defaults for ROS2
        ros_domain_id = 0
        if 'ROS_DOMAIN_ID' in os.environ:
            ros_domain_id = int(os.environ['ROS_DOMAIN_ID'])
        return NMD_DEFAULT_PORT + ros_domain_id
