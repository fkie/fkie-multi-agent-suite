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

import os
import subprocess

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
        # todo get 
        if os.path.isdir("../.git"):
            try:
                ps = subprocess.Popen(
                    ['git', 'describe', '--tags', '--dirty', '--always', '--abbrev=8'], stdout=subprocess.PIPE)
                output = ps.stdout.read()
                vers = output.decode('utf-8').strip()
                parts = vers.split('.', 3)
                if len(parts) < 3:
                    raise Exception('no version tag found in git')
                ps.wait()
                ps = subprocess.Popen(
                    ['git', 'show', '-s', '--format=%ci'], stdout=subprocess.PIPE)
                output = ps.stdout.read().split()
                if output:
                    date_str = output[0].decode('utf-8')
                else:
                    date_str = date.today().isoformat()
                ps.wait()
                VERSION = vers
                DATE = date_str
                rosNode.get_logger().info(f"detected version: {VERSION} {DATE}")
                return VERSION, DATE
            except Exception as _err:
                pass
                # print("git version detection error: %s" % err)
    except Exception:
        pass

    try:
        from fkie_mas_pylib import ros_pkg
        import xml.etree.ElementTree as ET
        from datetime import date
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
