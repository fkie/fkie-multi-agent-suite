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
