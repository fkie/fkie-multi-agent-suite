# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import sys


def utf8(s, errors='replace'):
    '''
    Converts string to unicode.
    '''
    if sys.version_info[0] <= 2:
        if isinstance(s, (str, buffer)):
            return unicode(s, 'utf-8', errors=errors)
        elif not isinstance(s, unicode):
            return unicode(str(s))
    elif isinstance(s, bytes):
        return s.decode('utf-8')
    elif not isinstance(s, str):
        return str(s)
    return s


def isstring(s):
    '''
    Small helper version to check an object is a string in a way that works
    for both Python 2 and 3
    '''
    try:
        return isinstance(s, basestring)
    except NameError:
        return isinstance(s, str)
