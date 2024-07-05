# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import roslib
import sys
import xml.dom.minidom as dom

from fkie_mas_daemon.strings import utf8

VERSION = "unknown"
DATE = ""


def detect_version(package):
    """
    Try to detect the current version from git, installed VERSION/DATE files or package.xml
    """
    global VERSION
    global DATE
    if VERSION != "unknown":
        return VERSION, DATE
    version = "unknown"
    date = ""
    try:
        ppath = roslib.packages.find_resource(package, "package.xml")
        if ppath:
            doc = dom.parse(ppath[0])
            version_tags = doc.getElementsByTagName("version")
            if version_tags:
                version = version_tags[0].firstChild.data
                version = version
            else:
                sys.stderr.write(
                    "version detection: no version tag in package.xml found!"
                )
        else:
            sys.stderr.write("version detection: package.xml not found!")
    except Exception as err:
        sys.stderr.write("version detection error: %s\n" % utf8(err))
    VERSION = version.decode() if (hasattr(version, "decode")) else version
    DATE = date.decode() if (hasattr(date, "decode")) else date
    return version, date
