# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

class LaunchDescription:

    def __init__(self, path='', masteruri='', host='', nodes=[], robot_descriptions=[], nodelets={}, associations={}):
        '''
        Description of the robot configured by this launch file.

         :param str path: path of the launch file.
         :param str masteruri: starts nodes of this file with specified ROS_MASTER_URI. If host is empty,
             the nodes are started on the host specified by hostname of the masteruri.
         :param str host: if not empty, the nodes of this launch file are launched on specified host.
         :param nodes: list of node names.
         :type nodes: [str]
         :param robot_descriptions: a list of capabilities :message:RobotDescription.
         :type robot_descriptions: [RobotDescription]
         :param nodelets: a dictionary with nodelets manager and controlled nodelet clients
         :type nodelets: {str: [str]}
         :param associations: a dictionary with associations of nodes
         :type associations: {str: [str]}
        '''
        self.path = path
        self.masteruri = masteruri
        self.host = host
        # create a new array to a void to fill a default one
        self.nodes = nodes if nodes else []
        # create a new array to a void to fill a default one
        self.robot_descriptions = robot_descriptions if robot_descriptions else []
        self.nodelets = nodelets if nodelets else {}
        self.associations = associations if associations else {}

    def __repr__(self):
        return "<%s[%s, masteruri: %s, host: %s], with %d nodes>" % (self.__class__, self.path, self.masteruri, self.host, len(self.nodes))

    def __str__(self):
        if self.nodes:
            return "%s [%s]" % (self.__repr__(), ','.join([str(node) for node in self.nodes]))
        return self.__repr__()


class RobotDescription:

    def __init__(self, machine='', robot_name='', robot_type='', robot_images=[], robot_descr='', capabilities=[]):
        '''
        Description of the robot configured by this launch file.

         :param str machine: the address of the host.
         :param str robot_name: robot name.
         :param str robot_type: type of the robot.
         :param robot_images: list of the images assigned to the robot.
         :type robot_images: [str]
         :param str robot_descr: some description.
         :param capabilities: a list of capabilities :message:Capability.
         :type capabilities: [Capability]
        '''
        self.machine = machine
        self.robot_name = robot_name
        self.robot_type = robot_type
        # create a new array to a void to fill a default one
        self.robot_images = robot_images if robot_images else []
        self.robot_descr = robot_descr
        # create a new array to a void to fill a default one
        self.capabilities = capabilities if capabilities else []

    def __repr__(self):
        return "<%s[%s], machine=%s, with %d capabilities>" % (self.__class__, self.robot_name, self.machine, len(self.capabilities))

    def __str__(self):
        if self.capabilities:
            return "%s [%s]" % (self.__repr__(), ','.join([str(cap) for cap in self.capabilities]))
        return self.__repr__()


class Capability:

    def __init__(self, name='', namespace='', cap_type='', images=[], description='', nodes=[]):
        '''
        Capabilities defined in launch file.

        :param str namespace: the ROS namespace of the capability.
        :param str name: the name of the capability.
        :param str type: the type of the capability.
        :param images: list of the images assigned to the this capability.
        :type images: [str]
        :param str description: the description of the capability.
        :param nodes: a list of nodes assigned to this group. The nodes are described by full ROS name (with namesspace).
        :type nodes: [str]
        '''
        self.namespace = namespace
        self.name = name
        self.type = cap_type
        # create a new array to a void to fill a default one
        self.images = images if images else []
        self.description = description
        # create a new array to a void to fill a default one
        self.nodes = nodes if nodes else []

    def __repr__(self):
        return "<%s[%s/%s], with %d nodes>" % (self.__class__, self.namespace, self.name, len(self.nodes))

    def __str__(self):
        if self.nodes:
            return "%s [%s]" % (self.__repr__(), ','.join([str(node) for node in self.nodes]))
        return self.__repr__()
