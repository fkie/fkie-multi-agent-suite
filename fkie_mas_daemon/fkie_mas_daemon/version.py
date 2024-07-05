# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

VERSION = 'unknown'
DATE = ''

def detect_version(rosNode, package):
    '''
    Try to detect the current version from git, installed VERSION/DATE files or package.xml from file created while build.
    '''
    global VERSION
    global DATE
    if VERSION != 'unknown':
        return VERSION, DATE

    try:
        from fkie_mas_pylib import ros_pkg
        import xml.etree.ElementTree as ET
        paths = ros_pkg.get_share_files_path_from_package('fkie_mas_daemon', 'package.xml')
        if paths:
            tree = ET.parse(paths[0])
            root = tree.getroot()
            for vers in root.findall('version'):
                VERSION = vers.text
                rosNode.get_logger().info(f"detected version: {VERSION} {DATE}")
                return VERSION, DATE
    except Exception:
        import traceback
        print(traceback.format_exc())
    return VERSION, DATE
