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


class GrpcTimeout(Exception):

    def __init__(self, remote, error):
        Exception.__init__(self)
        self.remote = remote
        self.error = error

    def __repr__(self):
        return "%s <%s>::%s" % (self.__class__, self.remote, repr(self.error))

    def __str__(self):
        return self.error
