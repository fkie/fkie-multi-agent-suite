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
from typing import Tuple
from typing import Union
from urllib.parse import urlparse


def equal_uri(url1: str, url2: str) -> bool:
    '''
    Removes to string after remove last slash character.
    '''
    return url1.rstrip(os.path.sep) == url2.rstrip(os.path.sep)


def get_port(url: str) -> Union[int, None]:
    '''
    Extracts the port from given url.

    :param str url: the url to parse
    :return: the port or `None`, if the url is `None` or `invalid`
    :rtype: int
    :see: http://docs.python.org/library/urlparse.html
    '''
    if url is None:
        return None
    if not url:
        return url
    result = None
    try:
        o = urlparse(url)
        result = o.port
        if result is None:
            res = url.split(':')
            if len(res) == 2:
                result = int(res[1])
    finally:
        return result


def split_uri(uri: str) -> Union[Tuple[str, str, int], None]:
    '''
    Splits URI or address into scheme, address and port and returns them as tuple.
    Scheme or tuple are empty if no provided.
    :param str uri: some URI or address
    :rtype: (str, str, int)
    '''
    (scheme, hostname, port) = ('', '', -1)
    if uri is None:
        return None
    if not uri:
        return uri
    try:
        o = urlparse(uri)
        scheme = o.scheme
        hostname = o.hostname
        port = o.port
    except AttributeError:
        pass
    if hostname is None:
        res = uri.split(':')
        if len(res) == 2:
            hostname = res[0]
            port = res[1]
        elif len(res) == 3:
            if res[0] == 'SHM':
                hostname = 'localhost'
                port = res[2]
            else:
                # split if more than one address
                hostname = res[1].strip('[]')
                port = res[2]
        elif len(res) == 4 and res[1] == 'SHM':
            hostname = 'localhost'
            port = res[3]
        else:
            hostname = uri
            port = -1
    try:
        port = int(port)
    except TypeError:
        port = -1
    return (scheme, hostname, port)
