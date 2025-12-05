# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from io import FileIO
import os
import re
import shlex
import subprocess

import json
from typing import List
from fkie_mas_pylib import ros_pkg
from fkie_mas_pylib.interface import SelfEncoder
from fkie_mas_pylib.interface.file_interface import FileItem
from fkie_mas_pylib.interface.file_interface import RosPackage
from fkie_mas_pylib.interface.file_interface import PathItem
from fkie_mas_pylib.interface.file_interface import LogPathItem
from fkie_mas_pylib.interface.file_interface import LogPathClearResult
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.system.screen import get_logfile
from fkie_mas_pylib.system.screen import get_ros_logfile
from fkie_mas_pylib.websocket.server import WebSocketServer


class FileServicer:
    FILE_CHUNK_SIZE = 1024

    def __init__(self, websocket: WebSocketServer):
        Log.info("Create ROS2 file manager servicer")
        # TODO: clear cache after detected change or time?
        self.CB_DIR_CACHE = {}
        websocket.register("ros.packages.get_list", self.getPackageList)
        websocket.register("ros.path.get_log_paths", self.getLogPaths)
        websocket.register("ros.path.clear_log_paths", self.clearLogPaths)
        websocket.register("ros.path.get_list", self.getPathList)
        websocket.register("ros.file.get", self.getFileContent)
        websocket.register("ros.file.save", self.saveFileContent)

    def stop(self):
        """ """
        pass

    def getPackageList(self, clear_cache: bool = False) -> List[RosPackage]:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.packages.get_list], force: {clear_cache}")
        if clear_cache:
            try:
                # try to find the setup.bash and update the environment
                # vars_to_save = ["ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"]
                # saved_environ = {}
                # for save_var in vars_to_save:
                #     if save_var in os.environ:
                #         saved_environ[save_var] = os.environ[save_var]
                setup_bash = ""
                if "AMENT_PREFIX_PATH" in os.environ:
                    first_ws_path = os.environ["AMENT_PREFIX_PATH"].split(":")[0]
                    setup_bash = os.path.join(os.path.dirname(first_ws_path), "setup.bash")
                    Log.info(f"{self.__class__.__name__}:   source {setup_bash}")
                if setup_bash and os.path.exists(setup_bash):
                    result = subprocess.run(
                        ["/bin/bash", "-c", f"source {shlex.quote(setup_bash)}; env"], capture_output=True, stdin=subprocess.DEVNULL, text=True)
                    if result.returncode == 0:
                        lines = result.stdout.split("\n")
                        env = {}
                        for line in lines:
                            line = line.strip()
                            if "=" in line:
                                key, value = line.split("=", 1)
                                # if value.find("/opt/ros") >= 0:
                                env[key] = value
                        os.environ.update(env)
                        # os.environ.update(saved_environ)
            except Exception as err:
                Log.warn(
                    f"{self.__class__.__name__}: Cannot reset package cache: {err}"
                )
        package_list: List[RosPackage] = []
        # fill the input fields
        ret = ros_pkg.get_packages(None)
        for name, path in ret.items():
            package = RosPackage(
                name=name, path=os.path.join(path, "share", name))
            package_list.append(package)
        return json.dumps(package_list, cls=SelfEncoder)

    def getLogPaths(self, nodes: List[str]) -> List[LogPathItem]:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.path.get_log_paths] for {nodes}"
        )
        result = []
        for node in nodes:
            namespace = None
            node_name = node

            namespace_search = re.search("/(.*)/", node_name)
            if namespace_search is not None:
                namespace = f"/{namespace_search.group(1)}"
                node_name = node.replace(f"/{namespace}/", "")

            screen_log = get_logfile(
                node=node_name, for_new_screen=True, namespace=namespace
            )
            ros_log = get_ros_logfile(node)
            log_path_item = LogPathItem(
                node,
                screen_log=screen_log,
                screen_log_exists=os.path.exists(screen_log),
                ros_log=ros_log,
                ros_log_exists=os.path.exists(ros_log),
            )
            result.append(log_path_item)
        return json.dumps(result, cls=SelfEncoder)

    def clearLogPaths(self, nodes: List[str]) -> List[LogPathClearResult]:
        Log.info(
            f"{self.__class__.__name__}: Request to [ros.path.clear_log_paths] for {nodes}"
        )
        result = []
        for node in nodes:
            namespace = None
            node_name = node

            namespace_search = re.search("/(.*)/", node_name)
            if namespace_search is not None:
                namespace = f"/{namespace_search.group(1)}"
                node_name = node.replace(f"/{namespace}/", "")

            screen_log = get_logfile(
                node=node_name, for_new_screen=True, namespace=namespace
            )
            ros_log = get_ros_logfile(node)
            resultDelete = True
            message = f"remove files: '{screen_log}' ,'{ros_log}'"
            Log.debug(
                f"{self.__class__.__name__}:   remove files: '{screen_log}' ,'{ros_log}'"
            )
            if (os.path.exists(screen_log)):
                try:
                    os.remove(screen_log)
                except OSError as error:
                    resultDelete = False
                    message += f"; Can not remove {screen_log}: {error}. "
            if (os.path.exists(ros_log)):
                try:
                    os.remove(ros_log)
                except OSError as error:
                    resultDelete = False
                    message += f"; Can not remove {ros_log}: {error}. "
            log_path_item = LogPathClearResult(
                node,
                result=resultDelete,
                message=message
            )
            result.append(log_path_item)
        return json.dumps(result, cls=SelfEncoder)

    def _glob(
        self,
        inputPath: str,
        recursive: bool = True,
        withHidden: bool = False,
        filter: List[str] = [],
    ) -> List[PathItem]:
        path_list: List[PathItem] = []
        dir_list: List[str] = []
        for name in os.listdir(inputPath):
            if not withHidden and name.startswith("."):
                continue
            filename = os.path.join(inputPath, name)
            if os.path.isfile(filename):
                path_list.append(
                    PathItem(
                        path=filename,
                        mtime=os.path.getmtime(filename),
                        size=os.path.getsize(filename),
                        path_type="file",
                    )
                )
            elif os.path.isdir(filename):
                if name not in filter:
                    path_list.append(
                        PathItem(
                            path=filename,
                            mtime=os.path.getmtime(filename),
                            size=os.path.getsize(filename),
                            path_type="dir",
                        )
                    )
                    if recursive:
                        dir_list.append(filename)
        # glob the directories at the end
        for filename in dir_list:
            path_list.extend(
                self._glob(
                    inputPath=filename,
                    recursive=recursive,
                    withHidden=withHidden,
                    filter=filter,
                )
            )
        return path_list

    def getPathList(self, inputPath: str, recursive: bool = False) -> List[PathItem]:
        Log.info(f"Request to [ros.path.get_list] for {inputPath}, recursive: {recursive}")
        path_list: List[PathItem] = self._glob(
            inputPath, recursive=recursive, withHidden=False, filter=["node_modules"]
        )

        return json.dumps(path_list, cls=SelfEncoder)

    def getFileContent(self, requestPath: str) -> FileItem:
        Log.info("Request to [ros.file.get] for %s" % requestPath)
        with FileIO(requestPath, "r") as outfile:
            mTime = os.path.getmtime(requestPath)
            fSize = os.path.getsize(requestPath)
            readonly = not os.access(requestPath, os.W_OK)
            content = outfile.readall()
            encoding = "utf-8"
            try:
                content = content.decode(encoding)
            except:
                content = content.hex()
                encoding = "hex"
            return json.dumps(
                FileItem(requestPath, mtime=mTime, size=fSize, readonly=readonly, value=content, encoding=encoding), cls=SelfEncoder
            )

    def saveFileContent(self, request_json: FileItem) -> int:
        # Covert input dictionary into a proper python object
        file = request_json
        # file = json.loads(
        #     json.dumps(request_json), object_hook=lambda d: SimpleNamespace(**d)
        # )
        Log.info("Request to [ros.file.save] for %s" % file.path)
        with FileIO(file.path, "w+") as outfile:
            content = file.value
            if file.encoding == "utf-8":
                content = content.encode("utf-8")
            elif file.encoding == "hex":
                content = bytes.fromhex(content)
            else:
                raise TypeError(f"unknown encoding {file.encoding}")
            bytesWritten = outfile.write(content)
            return json.dumps(bytesWritten, cls=SelfEncoder)
