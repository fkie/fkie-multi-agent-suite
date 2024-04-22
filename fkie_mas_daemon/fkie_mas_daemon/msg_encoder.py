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

import json
from typing import Any
from typing import Callable
from typing import Union
from typing import Tuple

import array
import numpy


class MsgEncoder(json.JSONEncoder):
    def __init__(self, *, skipkeys: bool = False, ensure_ascii: bool = True, check_circular: bool = True, allow_nan: bool = True, sort_keys: bool = False, indent: Union[int, str, None] = None, separators: Union[Tuple[str, str], None] = None, default: Union[Callable[..., Any], None] = None,
                 no_arr: bool = True,
                 no_str: bool = True) -> None:
        super().__init__(skipkeys=skipkeys, ensure_ascii=ensure_ascii, check_circular=check_circular,
                         allow_nan=allow_nan, sort_keys=sort_keys, indent=indent, separators=separators, default=default)
        self.no_arr = no_arr
        self.no_str = no_str

    def default(self, obj):
        result = {}
        fields = []
        if hasattr(obj, '__slots__'):
            fields = obj.__slots__
        if hasattr(obj, 'get_fields_and_field_types'):
            fields = obj.get_fields_and_field_types()
        for key in fields:
            item = getattr(obj, key)
            if isinstance(item, (bytes, array.array)):
                # obj_bytes = [byte for byte in obj]
                if (not self.no_arr):
                    obj_bytes = []
                    for byte in item:
                        if len(obj_bytes) >= 5:
                            break
                        obj_bytes.append(byte)
                    result[key] = ', '.join(
                        map(str, obj_bytes)) + f'...(of {len(item)} values)'
            elif isinstance(item, numpy.ndarray):
                if (not self.no_arr):
                    result[key] = item.tolist()
            elif isinstance(item, (list, dict, )):
                if (not self.no_arr):
                    result[key] = item
            elif isinstance(item, str):
                if (not self.no_str):
                    result[key] = item
            else:
                result[key] = item
        return result
