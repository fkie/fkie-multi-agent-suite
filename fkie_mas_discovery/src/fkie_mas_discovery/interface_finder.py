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

import time
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

import rospy
from .common import get_hostname
from fkie_mas_pylib.logging.logging import Log


def get_changes_topic(masteruri, wait=True, check_host=True):
    '''
    Search in publishers of ROS master for a topic with type `fkie_mas_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/MasterState.html>`_ and
    returns his name, if it runs on the local host. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the topics of type `fkie_mas_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/MasterState.html>`_

    :rtype: list of strings
    '''
    return _get_topic(masteruri, 'MasterState', wait, check_host)


def get_stats_topic(masteruri, wait=True, check_host=True):
    '''
    Search in publishers of ROS master for a topic with type LinkStatesStamped and
    returns his name, if it runs on the local host. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list of names of the topic with type `fkie_mas_discovery.msg.LinkStatesStamped <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/LinkStatesStamped.html>`_

    :rtype: list of strings
    '''
    return _get_topic(masteruri, 'LinkStatesStamped', wait, check_host)


def _get_topic(masteruri, ttype, wait=True, check_host=True):
    '''
    Search in publishers of ROS master for a topic with given type and
    returns his name, if it runs on the local host. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param ttype: the type of the topic

    :type ttype: str

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for equal hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list of names of the topic with type `fkie_mas_discovery.msg.LinkStatesStamped <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/LinkStatesStamped.html>`_

    :rtype: list of strings
    '''
    result = []
    while not result and not rospy.is_shutdown():
        master = xmlrpcclient.ServerProxy(masteruri)
        # get the system state to resolve the published nodes
        code, _, state = master.getSystemState(rospy.get_name())
        # read topic types
        code, msg, val = master.getPublishedTopics(rospy.get_name(), '')
        if code == 1:
            own_host = get_hostname(masteruri)
            nodes_host = []
            # search for a topic with type MasterState
            for topic, topic_type in val:
                if topic_type.endswith(ttype):
                    # get the name of the publisher node
                    for t, l in state[0]:
                        if topic == t:
                            if check_host:
                                # get the URI of the publisher node
                                for n in l:
                                    code, msg, val = master.lookupNode(
                                        rospy.get_name(), n)
                                    # only local publisher will be tacked
                                    if code == 1:
                                        hode_host = get_hostname(val)
                                        if hode_host == own_host:
                                            result.append(topic)
                                        else:
                                            nodes_host.append(hode_host)
                            else:
                                result.append(topic)
            if not result and wait:
                Log.warn(
                    f'Master_discovery node appear not to running @{own_host}, only found on {nodes_host}. Wait for topic with type "{ttype}" @{own_host}.')
                time.sleep(1)
        elif not result and wait:
            Log.warn(
                f'Cannot get published topics from ROS master: {code}, {msg}. Will keep trying!')
            time.sleep(1)
        if not wait:
            return result
    return result


def get_listmaster_service(masteruri, wait=True, check_host=True):
    '''
    Search in services of ROS master for a service with name ending by
    `list_masters` and returns his name, if it runs on the local host. Returns
    empty list if no service was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the service

    :type wait: boo

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the services ending with `list_masters`

    :rtype: list of strings
    '''
    return _get_service(masteruri, 'list_masters', wait, check_host)


def get_refresh_service(masteruri, wait=True, check_host=True):
    '''
    Search in services of ROS master for a service with name ending by
    `refresh` and returns his name, if it runs on the local host. Returns
    empty list if no service was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the service

    :type wait: boo

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the services ending with `refresh`

    :rtype: list of strings
    '''
    return _get_service(masteruri, 'refresh', wait, check_host)


def _get_service(masteruri, name, wait=True, check_host=True):
    '''
    Search in services of ROS master for a service with name ending by
    given name and returns his name, if it runs on the local host. Returns
    empty list if no service was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param name: the ending name of the service

    :type name: str

    :param wait: check every second for the service

    :type wait: bool

    :param check_host: check for equal hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the services ending with `refresh`

    :rtype: list of strings
    '''
    result = []
    while not result and not rospy.is_shutdown():
        master = xmlrpcclient.ServerProxy(masteruri)
        code, msg, val = master.getSystemState(rospy.get_name())
        if code == 1:
            pubs, subs, srvs = val
            own_host = get_hostname(masteruri)
            nodes_host = []
            # search for a service
            for srv, providers in srvs:
                if srv.endswith(name):
                    # only local service will be tacked
                    if check_host:
                        code, msg, val = master.lookupService(
                            rospy.get_name(), srv)
                        if code == 1:
                            hode_host = get_hostname(val)
                            if hode_host == own_host:
                                result.append(srv)
                            else:
                                nodes_host.append(hode_host)
                    else:
                        result.append(srv)
            if not result and wait:
                Log.warn(
                    f'mas-discovery node appear not to running @{own_host}, only found on {nodes_host}. Wait for service "{name}" @{own_host}.')
                time.sleep(1)
        elif not result and wait:
            Log.warn(f'cannot get state from ROS master: {code}, {msg}')
            time.sleep(1)
        if not wait:
            return result
    return result
