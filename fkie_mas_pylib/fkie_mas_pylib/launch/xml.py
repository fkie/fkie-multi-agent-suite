
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

from typing import Dict
from typing import List
from typing import Text
from typing import Tuple
from typing import Union

import os
import re
import sys
from xml.dom import minidom

from fkie_mas_pylib import ros_pkg
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.defines import EMPTY_PATTERN
from fkie_mas_pylib.defines import SEARCH_IN_EXT


class IncludedFile():

    def __init__(self, path_or_str: str, line_number: int, inc_path: str, exists: bool, raw_inc_path: str, rec_depth: int, args: Dict[str, str], size: int = 0):
        '''
        Representation of an included file found in given string or path of a file.

        :param str path_or_str: path of file or content where to search. If it is a path, the content will be read from file.
        :param int line_number: line number of the occurrence. If `unique` is True the line number is zero.
        :param str inc_path: resolved path.
        :param bool exists: True if resolved path exists.
        :param str raw_inc_path: representation of included file without resolved arg and find statements.
        :param int rec_depth: depth of recursion. if `unique` is True the depth is zero
        :param dict(str:str) args: a dictionary with arguments forwarded within include tag for 'inc_path'.
        '''
        self.path_or_str = path_or_str
        self.line_number = line_number
        self.inc_path = inc_path
        self.exists = exists
        self.raw_inc_path = raw_inc_path
        self.rec_depth = rec_depth
        self.args = args
        self.size = size
        self.unset_default_args = {}

    def __repr__(self):
        result = "<IncludedFile "
        result += " from=%s" % self.path_or_str
        result += " line_number=%d" % self.line_number
        result += " inc_path=%s" % self.inc_path
        result += " raw_inc_path=%s" % self.raw_inc_path
        result += " exists=%s" % self.exists
        result += " size=%d" % self.size
        result += " rec_depth=%d" % self.rec_depth
        result += " args=%s" % self.args
        result += " />"
        return result


def _replace_opt_env(content: str) -> Dict[str, str]:
    '''
    Searches in the content string for package include patterns.
    Returns a dictionary with package name and path suffix.
    '''
    result = content
    pattern = [r"\$\(optenv (.*?)\)",
               r"\$\(env (.*?)\)/([^ \"']*)"]
    env_pattern = re.compile('|'.join(pattern))
    groups = env_pattern.findall(content)
    for group in groups:
        for index in range(0, len(group)):
            env_name = group[index]
            if env_name:
                env_value = ""
                if env_name in os.environ:
                    env_value = os.environ[env_name]
                result = result.replace(f"$(optenv {env_name})", env_value)
                result = result.replace(f"$(env {env_name})", env_value)
    return result


def _find_include_tuple(content: str) -> Dict[str, str]:
    '''
    Searches in the content string for package include patterns.
    Returns a dictionary with package name and path suffix.
    '''
    result = {}
    pattern = [r"\$\(find (.*?)\)/([^ \"']*)",
               r"\$\(find-pkg-share (.*?)\)/([^ \"']*)",
               r"pkg:\/\/(.*?)/([^ \"']*)",
               r"package:\/\/(.*?)/([^ \"']*)"]
    pkg_pattern = re.compile('|'.join(pattern))
    groups = pkg_pattern.findall(_replace_opt_env(content))
    for group in groups:
        for index in range(0, len(group), 2):
            pkg_name = group[index]
            if pkg_name:
                path_suffix = ''
                if index + 1 < len(group):
                    path_suffix = group[index + 1]
                result[pkg_name] = path_suffix
    return result


def interpret_path(path: str, pwd: str = '.') -> str:
    '''
    Tries to determine the path of included file. The statement of $(find 'package') will be resolved.

    :param str path: the sting which contains the included path
    :param str pwd: current working path
    :return: `$(find 'package')` will be resolved. The prefixes `file://`, `package://` or `pkg://` are also resolved.
             Otherwise the parameter itself normalized by :py:func:`os.path.normpath` will be returned.
    :rtype: str
    '''
    result = path.strip().replace("$(dirname)", pwd)
    groups = _find_include_tuple(path)
    full_path_not_exists = ''
    for pkg_name, path_suffix in groups.items():
        path_suffix_stripped = path_suffix.strip(os.path.sep)
        # try to find the specific path in share
        try:
            paths = ros_pkg.get_share_files_path_from_package(
                pkg_name, path_suffix_stripped)
            if paths and os.path.exists(paths[0]):
                return paths[0]
        except Exception:
            import traceback
            Log.warn(
                f"search in install/devel space failed: {traceback.format_exc()}")

        pkg_path = ros_pkg.get_path(pkg_name)
        Log.debug(f"{result} got path for '{pkg_name}': {pkg_path}")
        # try to find first using ROS find_resource methods
        path_res = ros_pkg.get_ros_resource_from_package(
            pkg_path, path_suffix_stripped)
        if path_res and os.path.exists(path_res):
            return path_res
        if path_suffix.startswith('//'):
            path_suffix = path_suffix[2:]
        full_path = os.path.normpath(os.path.join(pkg_path, path_suffix))
        if os.path.exists(full_path):
            return full_path
        elif full_path:
            full_path_not_exists = full_path

    if path.startswith('file://'):
        result = path[7:]
    elif full_path_not_exists:
        return full_path_not_exists
    return os.path.normpath(os.path.join(pwd, result))


def replace_paths(text: str, pwd: str = '.') -> str:
    '''
    Like meth:interpret_path(), but replaces all matches in the text and retain other text.
    '''
    result = text
    path_pattern = re.compile(
        r"(\$\(dirname\)/)|(\$\(find-pkg-share .*?\)/)|(\$\(find .*?\)/)|(pkg:\/\/.*?/)|(package:\/\/.*?/)")
    for groups in path_pattern.finditer(text):
        for index in range(groups.lastindex):
            path = groups.groups()[index]
            if path:
                new_path = interpret_path(path, pwd)
                if os.path.exists(new_path):
                    new_path = os.path.realpath(new_path)
                result = result.replace(path.rstrip(os.path.sep), new_path)
    return result


def get_internal_args(content: str, path: str = '', resolve_args: Dict[str, str] = {}, only_default: bool = False) -> Dict[str, str]:
    '''
    Load the content with xml parser, search for arg-nodes.
    :return: a dictionary with detected arguments
    :rtype: {str: str}
    '''
    new_content = content
    try:
        resolve_args_intern = {}
        if sys.version_info < (3, 0):
            new_content = new_content.encode('utf-8')
        xml_nodes = minidom.parseString(
            new_content).getElementsByTagName('launch')
        for node in xml_nodes:
            for child in node.childNodes:
                if child.localName == 'arg' or child.localName == 'let' and child.hasAttributes():
                    aname = ''
                    aval = ''
                    add_arg = True
                    for argi in range(child.attributes.length):
                        arg_attr = child.attributes.item(argi)
                        if arg_attr.localName == 'name':
                            aname = arg_attr.value
                        elif arg_attr.localName in ['value', 'default']:
                            aval = arg_attr.value
                            # do not add this argument to the result list if value is set and 'only_default' is True
                            if only_default and arg_attr.localName == 'value':
                                add_arg = False
                    if aname and add_arg:
                        resolve_args_intern[aname] = replace_arg(aval, resolve_args)
    except Exception as err:
        import traceback
        Log.debug(
            f"error while get_internal_args for {path}: {traceback.format_exc()}")

    return resolve_args_intern


def get_internal_lets(content: str, resolve_args: Dict[str, str] = {}) -> Dict[str, str]:
    pattern = r'<let\s+name="([^"]+)"\s+value="([^"]+)"'
    matches = re.findall(pattern, content)

    let_dict = {name: replace_arg(value, resolve_args) for name, value in matches}
    return let_dict


def get_arg_names(content: str) -> List[str]:
    '''
    Searches for $(var <name>) statements and returns a list with <name>.
    :rtype: [str]
    '''
    result = []
    re_if = re.compile(r"\$\(arg.(?P<name>.*?)\)")
    for arg in re_if.findall(content):
        result.append(arg)
    return result


def replace_arg(content: str, resolve_args: Dict[str, str]) -> str:
    # test for if statement
    result = content
    for name, value in resolve_args.items():
        result = result.replace(f'$(var {name})', value)
    if (content != result):
        return replace_arg(result, resolve_args)
    return result


def _get_include_params(content: str, resolve_args: Dict[str, str]) -> List[str]:
    included_files = []
    try:
        xml_nodes = minidom.parseString(
            content).getElementsByTagName('include')
        for node in xml_nodes:
            if node.nodeType == node.ELEMENT_NODE and node.hasAttributes():
                filename = ''
                for ai in range(node.attributes.length):
                    attr = node.attributes.item(ai)
                    if attr.localName == 'file':
                        filename = attr.value
                inc_args = node.getElementsByTagName('arg')
                resolved_inc_args = {}
                for inc_arg in inc_args:
                    if inc_arg.nodeType == node.ELEMENT_NODE and inc_arg.hasAttributes():
                        aname = ''
                        aval = ''
                        skip = False
                        for argi in range(inc_arg.attributes.length):
                            arg_attr = inc_arg.attributes.item(argi)
                            if arg_attr.localName == 'name':
                                aname = arg_attr.value
                            elif arg_attr.localName in ['value', 'default']:
                                aval = replace_arg(arg_attr.value, resolve_args)
                            elif arg_attr.localName == 'if':
                                val = replace_arg(arg_attr.value, resolve_args)
                                skip = val in ['false', '0']
                            elif arg_attr.localName == 'unless':
                                val = replace_arg(arg_attr.value, resolve_args)
                                skip = val in ['true', '1']
                        if aname and not skip:
                            aval = replace_paths(aval)
                            resolved_inc_args[aname] = aval
                if filename:
                    included_files.append((filename, resolved_inc_args))
    except Exception as err:
        print(f"_get_include_params reports: {err}")
        Log.debug(f"_get_include_params reports: {err}")
    return included_files


def find_included_files(string: str,
                        recursive: bool = True,
                        unique: bool = False,
                        search_in_ext: List[str] = SEARCH_IN_EXT,
                        resolve_args: Dict[str, str] = {},
                        unique_files: List[str] = None,
                        rec_depth: int = 0,
                        filename: str = None) -> List[IncludedFile]:
    '''
    If the `string` parameter is a valid file the content of this file will be parsed.
    In other case the `string` is parsed to find included files.

    :param str string: Path to an exists file or test with included file.
    :param bool recursive: parse also found included files (Default: True)
    :param bool unique: returns the same files once (Default: False)
    :param search_in_ext: file extensions to search in
    :type search_in_ext: [str]
    :param resolve_args: dictionary with arguments to resolve arguments in path names
    :type resolve_args: {str, str}
    :return: Returns an iterator with IncludedFile-class
    :rtype: iterator with IncludedFile
    '''
    pattern = [r"\s*(\$\(find-pkg-share .*?\)[^\n\t\"]*)",
               r"\s*(\$\(find .*?\)[^\n\t\"]*)",
               r"\s*(\$\(dirname\)[^\n\t\"]*)",
               r"\s*(pkg:\/\/.*?/[^\n\t\"]*)",
               r"\s*(package:\/\/.*?/[^\n\t\"]*)",
               r"textfile=\"(.*?)\n\t\"",
               r"binfile=\"(.*?)\n\t\"",
               r"file=\"(.*?)\n\t\""]
    re_filelist = re.compile('|'.join(pattern))
    pwd = '.'
    content = string
    content_info = 'content'
    # read file content if file exists
    if os.path.exists(string) and not os.path.isdir(string):
        pwd = os.path.dirname(string)
        content_info = string
        with open(string, 'r') as f:
            content = f.read()
            # replace XML comments by the same count of NEWLINES
            comment_pattern = re.compile(r"<!--.*?-->", re.DOTALL)
            match = comment_pattern.search(content)
            while match is not None:
                count_nl = content[match.start():match.end()].count('\n')
                content = content[:match.start()] + '\n' * \
                    count_nl + content[match.end():]
                match = comment_pattern.search(content, match.start())
            # HACK: if we found the include pattern in YAML file....
            comment_pattern = re.compile(r"\s*(#.*?[^\n\t\"]*)", re.DOTALL)
            match = comment_pattern.search(content)
            while match is not None:
                tt = content[match.start():match.end()]
                count_nl = content[match.start():match.end()].count('\n')
                content = content[:match.start()] + '\n' * \
                    count_nl + content[match.end():]
                match = comment_pattern.search(content, match.start())
    # use dirname if given filename if valid
    if filename is not None:
        pwd = os.path.dirname(filename)
        if '://' in pwd:
            pwd = re.sub(r"^.*://[^/]*", "", pwd)
    inc_files_forward_params = []
    # replace the arguments and detect arguments for include-statements
    internal_args = {}
    if (string.endswith('.launch') or string.find('.launch.') > 0):
        # Determine the arguments defined inside the launch file
        internal_args = get_internal_args(content, filename, resolve_args, True)
        # Determine the parameters that are used for loading
        inc_files_forward_params = _get_include_params(content, resolve_args)
    my_unique_files = unique_files if unique_files is not None else list()
    # search for include pattern in the content without comments
    for groups in re_filelist.finditer(content):
        if groups.lastindex is None:
            continue
        for index in range(groups.lastindex):
            fname = remove_after_space(groups.groups()[index])
            rawname = fname
            if fname:
                try:
                    forward_params = {}
                    if inc_files_forward_params and inc_files_forward_params[0][0] == fname:
                        forward_params = inc_files_forward_params[0][1]
                        inc_files_forward_params.pop(0)
                    resolve_args_all = dict(resolve_args)
                    resolve_args_all.update(forward_params)
                    try:
                        # try to resolve path
                        fname = replace_arg(fname, resolve_args_all)
                        fname = replace_arg(fname, internal_args)
                        if fname.find('$(var ') > -1:
                            idx = content.find(groups.groups()[index])
                            if idx > -1:
                                internal_lets = get_internal_lets(content[0:idx], resolve_args_all)
                                fname = replace_arg(fname, internal_lets)
                        if fname.find('$(var ') == -1:
                            # do not try to resolve if not all args are replaced
                            fname = interpret_path(fname, pwd)
                    except Exception as err:
                        import traceback
                        print(traceback.format_exc())
                        Log.warn(f"Interpret file failed: {err}")
                    if os.path.isdir(fname):
                        fname = ''
                    exists = os.path.isfile(fname)
                    if fname:
                        publish = not unique or (
                            unique and fname not in my_unique_files)
                        if publish:
                            my_unique_files.append(fname)
                            # transform found position to line number
                            content_tmp = content
                            if sys.version_info < (3, 0):
                                content_tmp = content.decode('utf-8')
                            position = content_tmp.count(
                                "\n", 0, groups.start()) + 1
                            yield IncludedFile(string, position, fname, exists, rawname, rec_depth, resolve_args_all)
                    # for recursive search
                    if exists:
                        if recursive:
                            try:
                                ext = os.path.splitext(fname)
                                if ext[1] in search_in_ext:
                                    for res_item in find_included_files(fname, recursive, False, search_in_ext, resolve_args_all, rec_depth=rec_depth + 1, filename=fname):
                                        publish = not unique or (
                                            unique and res_item.inc_path not in my_unique_files)
                                        if publish:
                                            my_unique_files.append(
                                                res_item.inc_path)
                                            yield res_item
                            except Exception as e:
                                Log.warn(
                                    f"Error while recursive search for include pattern in {fname}: {e}")
                except Exception as e:
                    Log.warn(
                        f"Error while parse {content_info} for include pattern: {e}")


def remove_after_space(filename: str) -> str:
    result = filename
    if filename:
        idx_whitespace = -1
        idx = len(filename) - 1
        for c in reversed(filename):
            if c == '.':
                break
            elif c == ' ':
                idx_whitespace = idx
            idx -= 1
        if idx > -1 and idx_whitespace > -1:
            result = filename[:idx_whitespace]
    return result
