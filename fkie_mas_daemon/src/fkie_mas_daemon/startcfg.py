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

from fkie_mas_daemon.strings import utf8
from fkie_mas_pylib.system.host import get_hostname


class StartConfig():

    def __init__(self, package, binary):
        '''
        :param str host: master uri from host where to run the node. Masteruri is used for cases where NMD uri needed.
        '''
        self.package = package
        self.binary = binary
        self.config_path = ''
        self.binary_path = ''
        self.name = ''
        self.namespace = ''
        self.fullname = ''
        self.prefix = ''
        self.cwd = ''
        self.env = {}
        self.remaps = {}
        self.params = {}
        self.clear_params = []
        self.args = []
        self.masteruri = None
        self.host = None
        self.loglevel = ''
        self.logformat = ''
        self.respawn = False
        self.respawn_delay = 30
        self.respawn_max = 0
        self.respawn_min_runtime = 0

    def __repr__(self):
        params = "name=%s" % self.name
        params += ", ns=%s" % self.namespace
        params += ", package=%s" % self.package
        params += ", binary=%s" % self.binary
        params += ", prefix=%s" % self.prefix
        params += ", cwd=%s" % self.cwd
        params += ", masteruri=%s" % self.masteruri
        params += ", host=%s" % self.host
        params += ", loglevel=%s" % self.loglevel
        params += ", respawn=%s" % self.respawn
        return "<StartConfig %s/>" % params

    @property
    def hostname(self):
        '''
        :return: host name from host_masteruri if it is not None.
        '''
        if self.host:
            return get_hostname(self.host)
        return None

    @property
    def nmduri(self):
        '''
        :return: the nmd uri where to launch the node from host_masteruri if it is not None.
        '''
        return None

    # def _msg_type(self, value):
    #     valtype = type(value)
    #     if valtype == int:
    #         return INT32
    #     if valtype == float:
    #         return DOUBLE
    #     if valtype == bool:
    #         return BOOL
    #     if valtype == list:
    #         return LIST
    #     return STRING

    # @classmethod
    # def _from_msg_type(cls, value, value_type):
    #     if value_type == INT32:
    #         return int(value)
    #     if value_type == DOUBLE:
    #         return float(value)
    #     if value_type == BOOL:
    #         return value.lower() in ("yes", "true", "t", "1")
    #     if value_type == LIST:
    #         try:
    #             return eval(value)
    #         except Exception:
    #             return []
    #     return value

    # def to_msg(self):
    #     msg = lmsg.StartConfig(package=self.package, binary=self.binary)
    #     self.fill_msg(msg)
    #     return msg

    # def fill_msg(self, msg):
    #     msg.package = self.package
    #     msg.binary = self.binary
    #     if self.binary_path:
    #         msg.binary_path = self.binary_path
    #     if self.name:
    #         msg.name = self.name
    #     if self.namespace:
    #         msg.namespace = self.namespace
    #     if self.fullname:
    #         msg.fullname = self.fullname
    #     if self.prefix:
    #         msg.prefix = self.prefix
    #     if self.cwd:
    #         msg.cwd = self.cwd
    #     if self.env:
    #         msg.env.extend([lmsg.Argument(name=name, value=value)
    #                         for name, value in self.env.items()])
    #     if self.remaps:
    #         msg.remaps.extend([lmsg.Remapping(from_name=name, to_name=value)
    #                            for name, value in self.remaps.items()])
    #     if self.params:
    #         msg.params.extend([lmsg.Argument(name=name, value=utf8(
    #             value), value_type=self._msg_type(value)) for name, value in self.params.items()])
    #     if self.clear_params:
    #         msg.clear_params.extend(self.clear_params)
    #     if self.args:
    #         msg.args.extend(self.args)
    #     if self.masteruri:
    #         msg.masteruri = self.masteruri
    #     if self.host:
    #         msg.host = self.host
    #     msg.loglevel = self.loglevel
    #     msg.respawn = self.respawn
    #     msg.respawn_delay = self.respawn_delay
    #     msg.respawn_max = self.respawn_max
    #     msg.respawn_min_runtime = self.respawn_min_runtime

    @classmethod
    def from_msg(cls, msg):
        startcfg = StartConfig(msg.package, msg.binary)
        startcfg.binary_path = msg.binary_path
        startcfg.name = msg.name
        startcfg.namespace = msg.namespace
        startcfg.fullname = msg.fullname
        startcfg.prefix = msg.prefix
        startcfg.cwd = msg.cwd
        startcfg.env = {env.name: env.value for env in msg.env}
        startcfg.remaps = {
            remap.from_name: remap.to_name for remap in msg.remaps}
        startcfg.params = {param.name: cls._from_msg_type(
            param.value, param.value_type) for param in msg.params}
        startcfg.clear_params = list(msg.clear_params)
        startcfg.args = list(msg.args)
        startcfg.masteruri = msg.masteruri
        startcfg.host = msg.host
        startcfg.loglevel = msg.loglevel
        startcfg.respawn = msg.respawn
        startcfg.respawn_delay = msg.respawn_delay
        startcfg.respawn_max = msg.respawn_max
        startcfg.respawn_min_runtime = msg.respawn_min_runtime
        return startcfg
