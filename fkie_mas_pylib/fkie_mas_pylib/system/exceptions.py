# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from typing import Text


class ListSelectionRequest(Exception):
    ''' '''

    def __init__(self, choices, error):
        Exception.__init__(self)
        self.choices = choices
        self.error = error

    def __repr__(self):
        return "%s <choices=%s>::%s" % (self.__class__, str(self.choices), repr(self.error))

    def __str__(self):
        return self.error


class BinarySelectionRequest(ListSelectionRequest):
    pass


class LaunchSelectionRequest(ListSelectionRequest):
    pass


class ParamSelectionRequest(ListSelectionRequest):
    pass


class StartException(Exception):
    pass


class AlreadyOpenException(Exception):

    def __init__(self, path, error):
        Exception.__init__(self)
        self.path = path
        self.error = error

    def __repr__(self):
        return "%s <path=%s>::%s" % (self.__class__, self.path, repr(self.error))

    def __str__(self):
        return self.error


class ResourceNotFound(AlreadyOpenException):
    pass


class RemoteException(Exception):

    def __init__(self, code, error):
        Exception.__init__(self)
        self.code = code
        self.error = error

    def __repr__(self):
        return "%s <code=%s>::%s" % (self.__class__, self.code, repr(self.error))

    def __str__(self):
        return self.error


class ConnectionException(Exception):

    def __init__(self, remote, error):
        Exception.__init__(self)
        self.remote = remote
        self.error = error

    def __repr__(self):
        return "%s %s::%s" % (self.__class__, self.remote, repr(self.error))

    def __str__(self):
        return self.error


