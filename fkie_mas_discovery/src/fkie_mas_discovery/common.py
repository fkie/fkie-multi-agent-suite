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
import re

try:
    from urlparse import urlparse
except ImportError:
    from urllib.parse import urlparse

import roslib.names
import rospy

from fkie_mas_pylib.defines import EMPTY_PATTERN
from fkie_mas_pylib.logging.logging import Log


IP4_PATTERN = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}")


def get_hostname(url):
    '''
    Extracts the hostname from given url.

    :param str url: the url to parse
    :return: the hostname or `None`, if the url is `None` or `invalid`
    :rtype: str
    :see: http://docs.python.org/library/urlparse.html
    '''
    if url is None:
        return None
    o = urlparse(url)
    hostname = o.hostname
    if hostname is None:
        div_idx = url.find(':')
        if div_idx > -1:
            hostname = url[0:div_idx]
        else:
            hostname = url
    return hostname


def resolve_url(interface_url, pwd='.'):
    '''
    The supported URL begins with `file:///`, `package://` or `pkg://`.
    The package URL will be resolved to a valid file path. If the file is in a
    subdirectory, you can replace the subdirectory by `///`.

    E.g.: `package://fkie_mas_discovery///mas_discovery.launch`

    :raise ValueError: on invalid URL or not existent file
    :return: the file path
    '''
    filename = ''
    if interface_url:
        if interface_url.startswith('file://'):
            filename = interface_url[7:]
        elif interface_url.startswith('package://') or interface_url.startswith('pkg://'):
            length = 6 if interface_url.startswith('pkg://') else 10
            pkg_name, _, pkg_path = interface_url[length:].partition('/')
            if pkg_path.startswith('//'):
                paths = roslib.packages.find_resource(
                    pkg_name, pkg_path.strip('/'))
                if len(paths) > 0:
                    # if more then one launch file is found, take the first one
                    filename = paths[0]
            else:
                pkg_dir = roslib.packages.get_pkg_dir(pkg_name)
                filename = os.path.join(pkg_dir, pkg_path)
        else:
            filename = interface_url
        if filename == '.':
            filename = ''
        if filename:
            filename = os.path.join(pwd, filename)
            if not os.path.exists(filename):
                raise ValueError(
                    'unsupported interface URL or interface file not found: ' + interface_url)
    return filename


def read_interface(interface_file):
    '''
    Reads the given file. You can use :mod:`fkie_mas_discovery.common.resolve_url()`
    to resolve an URL to a file.

    :param str interface_file: the file containing the interface.
    :raise ValueError: on error while read interface
    :return: directory with content of the given file
    '''
    data = {}
    with open(interface_file, 'r') as f:
        iface = f.read()
        # parse Interface file / YAML text
        # - lazy import
        import yaml
        try:
            data = yaml.load(iface)
            if data is None:
                data = {}
        except yaml.MarkedYAMLError as e:
            if not interface_file:
                raise ValueError(
                    "Error within YAML block:\n\t%s\n\nYAML is:\n%s" % (str(e), iface))
            else:
                raise ValueError("file %s contains invalid YAML:\n%s" %
                                 (interface_file, str(e)))
        except Exception as e:
            if not interface_file:
                raise ValueError(
                    "invalid YAML: %s\n\nYAML is:\n%s" % (str(e), iface))
            else:
                raise ValueError("file %s contains invalid YAML:\n%s" %
                                 (interface_file, str(e)))
    return data


def create_pattern(param, data, has_interface, default=[], mastername=''):
    '''
    Create and compile the regular expression for given parameter. The data is
    taken from `data`. If the data was read from the interface file, then you have
    to set the `has_interface` to True. If `has_interface` is False, the data will
    be ignored and the parameter will be read from ROS parameter server.
    If resulting value is an empty list, `\\\\b` (http://docs.python.org/2/library/re.html)
    will be added to the pattern as `EMPTY_PATTERN`.

    :param str param: parameter name
    :param dict data: The dictionary, which can contain the parameter name and value.
                      The `data` will be ignored, if `has_interface` is `False`.
    :param bool has_interface: `True`, if valid data is available.
    :param list default: Default value will be added to the data
    :return: the compiled regular expression
    :rtype: The result of `re.compile()`
    '''
    def_list = default
    if has_interface:  # read the parameter from the sync interface data
        if param in data and data[param]:
            for item in data[param]:
                _parse_value(item, mastername, def_list)
    else:  # reads the patterns from the ROS parameter server
        rp = get_ros_param('~%s' % param, [])
        _parse_value(rp, mastername, def_list)
        # reads the mastername specific parameters
        if mastername:
            rph = get_ros_param(
                '~%s' % roslib.names.ns_join(mastername, param), [])
            if isinstance(rp, list):
                def_list[len(def_list):] = rph
            else:
                def_list.append(rph)
    def_list = list(set(def_list))
    return gen_pattern(def_list, param, print_info=True, mastername=mastername)


def get_ros_param(name, default):
    try:
        return rospy.get_param(name, default)
    except Exception:
        pass
    return default


def _parse_value(value, mastername, def_list):
    if isinstance(value, dict):
        # this are mastername specific remapings
        if mastername and mastername in value:
            if isinstance(value[mastername], list):
                def_list[len(def_list):] = value[mastername]
            else:
                def_list.append(value[mastername])
    elif isinstance(value, list):
        for item in value:
            if isinstance(item, dict):
                # this are mastername specific remapings
                if mastername and mastername in item:
                    if isinstance(item[mastername], list):
                        def_list[len(def_list):] = item[mastername]
                    else:
                        def_list.append(item[mastername])
            else:
                def_list.append(item)
    else:
        def_list.append(value)


def gen_pattern(filter_list, name, print_info=True, mastername=None):
    if print_info:
        if mastername is not None and mastername:
            Log.info(f'[{mastername}] {name}: {filter_list}')
        else:
            Log.info(f'{name}: {filter_list}')
    def_list = [''.join(['\A', n.strip().replace('*', '.*'), '\Z'])
                for n in filter_list]
    if def_list:
        return re.compile('|'.join(def_list), re.I)
    return EMPTY_PATTERN


def is_empty_pattern(re_object):
    '''
    Returns the value of `EMPTY_PATTERN`.
    '''
    return re_object == EMPTY_PATTERN
