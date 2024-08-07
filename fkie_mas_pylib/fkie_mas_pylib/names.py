
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from typing import Text

from fkie_mas_pylib.defines import PRIV_NAME
from fkie_mas_pylib.defines import SEP


def ns_join(ns: Text, name: Text) -> Text:
    """
    Join a namespace and name. If name is unjoinable (i.e. ~private or
    /global) it will be returned without joining

    :param str ns: namespace ('/' and '~' are both legal). If ns is the empty string, name will be returned.
    :param str name: a legal name
    :return: name concatenated to ns, or name if it is unjoinable.
    :rtype: str
    """
    if not name:
        return name
    if name[0] == SEP or name[0] == PRIV_NAME:
        return name
    if ns == PRIV_NAME:
        return PRIV_NAME + name
    if not ns:
        return name
    if ns[-1] == SEP:
        return ns + name
    return ns + SEP + name


def namespace(name: Text, with_sep_suffix: bool = True, global_on_none: bool = False, raise_err_on_none: bool = True) -> Text:
    """
    Get the namespace of name. The namespace is returned with a
    trailing slash in order to favor easy concatenation and easier use
    within the global context.

    :param str name: name to return the namespace of. Must be a legal
        name. NOTE: an empty name will return the global namespace.
    :return str: Namespace of name. For example, '/wg/node1' returns '/wg/'. The
        global namespace is '/'. 
    :rtype: str
    :raise ValueError: if name is invalid
    """
    if name is None or not name:
        if global_on_none:
            return SEP
        elif raise_err_on_none:
            raise ValueError('name')
        else:
            return ''
    elif name[-1] == SEP:
        name = name[:-1]
    offset = 1 if with_sep_suffix else 0
    return name[:name.rfind(SEP)+offset] or SEP


def basename(p) -> str:
    """
    Returns the final component of a node name
    """
    if p:
        i = p.rfind(SEP) + 1
        return p[i:]
    return ''
