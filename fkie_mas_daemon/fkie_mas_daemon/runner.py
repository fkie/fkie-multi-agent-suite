from ament_index_python import get_package_prefix
from ament_index_python import PackageNotFoundError
import subprocess
import os
import sys
from fkie_mas_pylib.interface.launch_interface import RunNode
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system import screen
from fkie_mas_pylib.system.supervised_popen import SupervisedPopen


class PackageNotFound(Exception):

    def __init__(self, package_name):
        self.package_name = package_name


class MultipleExecutables(Exception):

    def __init__(self, paths):
        self.paths = paths


class Runner():

    def get_prefix_path(self, package_name):
        try:
            prefix_path = get_package_prefix(package_name)
        except (PackageNotFoundError, ValueError):
            return None
        return prefix_path

    def get_executable_paths(self, *, package_name):
        prefix_path = self.get_prefix_path(package_name)
        if prefix_path is None:
            raise PackageNotFound(package_name)
        base_path = os.path.join(prefix_path, 'lib', package_name)
        executable_paths = []
        for dirpath, dirnames, filenames in os.walk(base_path):
            # ignore folder starting with .
            dirnames[:] = [d for d in dirnames if d[0] not in ['.']]
            dirnames.sort()
            # select executable files
            for filename in sorted(filenames):
                path = os.path.join(dirpath, filename)
                if os.access(path, os.X_OK):
                    executable_paths.append(path)
        return executable_paths

    def get_executable_path(self, *, package_name, executable_name):
        paths = self.get_executable_paths(package_name=package_name)
        paths2base = {}
        for p in paths:
            basename = os.path.basename(p)
            if basename == executable_name:
                # pick exact match
                paths2base[p] = basename
            elif sys.platform == 'win32':
                # check extensions listed in PATHEXT for match without extension
                pathext = os.environ.get('PATHEXT', '').lower().split(os.pathsep)
                ext = os.path.splitext(basename)[1].lower()
                if ext in pathext and basename[:-len(ext)] == executable_name:
                    # pick match because of known extension
                    paths2base[p] = basename
        if not paths2base:
            return None
        if len(paths2base) > 1:
            raise MultipleExecutables(paths2base.keys())
        return list(paths2base.keys())[0]

    def run_node(self, node: RunNode):
        cmd = getattr(node, "opt_binary", "")
        if not cmd:
            executable_path = self.get_executable_path(package_name=node.package_name, executable_name=node.executable)
            cmd = [executable_path]
        else:
            executable_path = cmd
            cmd = [cmd]

        # on Windows Python scripts are invokable through the interpreter
        if os.name == 'nt' and path.endswith('.py'):
            cmd.insert(0, sys.executable)

        node_prefix = getattr(node, "prefix", "")
        if node_prefix:
            cmd = node_prefix + cmd

        node_name = getattr(node, "name", "")
        node_namespace = getattr(node, "namespace", "")
        node_loglevel = getattr(node, "loglevel", "")
        node_params = getattr(node, "params", "")
        if node_name or node_namespace or node_loglevel or node_params:
            cmd = cmd + ['--ros-args']

        if node_loglevel:
            cmd = cmd + ['--log-level'] + [node_loglevel]

        if node_namespace:
            cmd = cmd + ['-r __ns:=' + node_namespace]

        if node_name:
            cmd = cmd + ['-r __node:=' + node_name]

        if node_params:
            cmd = cmd + node_params


        unique_name = os.path.join(node_namespace, node_name)
        if not unique_name or unique_name == "/":
            unique_name = getattr(node, "executable", "")
        if not unique_name:
            unique_name = os.path.basename(getattr(node, "opt_binary", ""))
        if not unique_name:
            raise Exception("no name provided")

        screen_prefix = screen.get_cmd(unique_name)
        # set environment
        new_env = os.environ.copy()
        # if node.env:
        #     new_env.update(node.env)
        # set display variable to local display
        if 'DISPLAY' in new_env:
            if not new_env['DISPLAY'] or new_env['DISPLAY'] == 'remote':
                del new_env['DISPLAY']
        else:
            new_env['DISPLAY'] = ':0.0'
        # add environment from launch
        # if node.additional_env:
        #     new_env.update(dict(node.additional_env))
        if node_namespace:
            new_env['ROS_NAMESPACE'] = node_namespace
        # set logging
        node_logformat = getattr(node, "logformat", "")
        if node_logformat:
            new_env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = node_logformat
        # start
        Log.info(f"{screen_prefix} {' '.join(cmd)}")
        Log.debug(f"environment while run node '{unique_name}': '{new_env}'")
        open = SupervisedPopen(' '.join([screen_prefix, ' '.join(cmd)]), shell=True, env=new_env,
                        object_id=f"run_node_{unique_name}", description=f"Run [{node.package_name}]{node.executable}")
        # result_err = None
        # if open.stdout is not None:
        #     result_err = open.stdout.read()
        # if result_err:
        #     Log.warn(result_err)
        return executable_path
