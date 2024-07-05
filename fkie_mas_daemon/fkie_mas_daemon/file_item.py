# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


EFILE_CHANGED = 125
''':var EFILE_CHANGED: file changed in meantime.'''
EFILE_REMOVED = 126
''':var EFILE_REMOVED: file removed in meantime.'''


class FileItem(object):
    FILE = 0
    DIR = 1
    SYMLINK = 2
    PACKAGE = 3

    def __init__(self, path, path_type, size, mtime):
        self.path = path
        self.type = path_type
        self.size = size
        self.mtime = mtime
