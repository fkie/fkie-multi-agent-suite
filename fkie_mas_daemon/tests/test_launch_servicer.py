# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import unittest
import time
import json
from types import SimpleNamespace

from fkie_mas_daemon.launch_servicer import LaunchServicer
from fkie_mas_daemon.launch_description import RobotDescription, Capability
from fkie_mas_pylib.interface.launch_interface import LaunchIncludedFilesRequest
from fkie_mas_pylib.launch import xml

PKG = 'fkie_mas_daemon'



class TestLaunchServicer(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.test_get_content_path = "%s/content_test.txt" % os.getcwd()
        self.current_pose = 0

    def tearDown(self):
        try:
            os.remove(self.test_get_content_path)
        except Exception:
            pass

    def test_get_loaded_files(self):
        ls = LaunchServicer(monitor_servicer=None, loop=None, test_env=True)
        content_response = ls.get_list()
        content_response = json.loads(
            content_response, object_hook=lambda d: SimpleNamespace(**d)
        )
        items = [response for response in content_response]
        self.assertEqual(
            len(items), 0, "The list of loaded launch files on startup is not empty!")
#            self.assertEqual(resp.ack.size, self.current_pose, "incorrect transferred file size: %d, expected: %d" % (resp.ack.size, self.current_pose))

    # def test_get_included_files(self):
    #     ls = LaunchServicer(monitor_servicer=None, loop=None, test_env=True)
    #     path = xml.interpret_path(
    #         "$(find fkie_mas_daemon)/tests/resources/include_dummy.launch")
    #     response_stream = ls.get_included_files(lmsg.IncludedFilesRequest(
    #         path=path, recursive=True, unique=True, pattern=[]), DummyContext())
    #     file_list = [response for response in response_stream]
    #     self.assertEqual(len(
    #         file_list), 5, "Count of unique included, recursive  files is wrong, got: %d, expected: %d" % (len(file_list), 5))
    #     response_stream = ls.GetIncludedFiles(lmsg.IncludedFilesRequest(
    #         path=path, recursive=False, unique=True, pattern=[]), DummyContext())
    #     file_list = [response for response in response_stream]
    #     self.assertEqual(len(
    #         file_list), 3, "Count of unique included files while not recursive search is wrong, got: %d, expected: %d" % (len(file_list), 3))
    #     response_stream = ls.GetIncludedFiles(lmsg.IncludedFilesRequest(
    #         path=path, recursive=False, unique=False, pattern=[]), DummyContext())
    #     file_list = [response for response in response_stream]
    #     self.assertEqual(len(
    #         file_list), 6, "Count of not recursive, not unique included files is wrong, expected: %d, got: %d" % (6, len(file_list)))
    #     response_stream = ls.GetIncludedFiles(lmsg.IncludedFilesRequest(
    #         path=path, recursive=True, unique=False, pattern=[]), DummyContext())
    #     file_list = [response for response in response_stream]
    #     self.assertEqual(len(file_list), 10, "Count of included files is wrong, got: %d, expected: %d" % (
    #         len(file_list), 10))
    #     self.assertEqual(file_list[0].linenr, 6, "Wrong line number of first included file, got: %d, expected: %d" % (
    #         file_list[0].linenr, 6))
    #     self.assertEqual(file_list[1].linenr, 4, "Wrong line number of second included file, got: %d, expected: %d" % (
    #         file_list[1].linenr, 4))
    #     self.assertEqual(file_list[2].linenr, 10, "Wrong line number of third included file, got: %d, expected: %d" % (
    #         file_list[2].linenr, 10))

    # def test_load_launch(self):
    #     ls = LaunchServicer(monitor_servicer=None, loop=None, test_env=True)
    #     path = ''
    #     args = {}
    #     request_args = True
    #     response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_mas_daemon',
    #                                                     launch='no_file.launch', path=path, request_args=request_args), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'),
    #                      "wrong status code if launch file not exists, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'), response.status.error_msg))
    #     response = ls.LoadLaunch(lmsg.LoadLaunchRequest(
    #         package='unknonwn_package', launch='no_file.launch', path=path, request_args=request_args), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'),
    #                      "wrong status code if package not exists, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'), response.status.error_msg))
    #     response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_mas_daemon',
    #                                                     launch='description_example.launch', path=path, request_args=request_args), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('MULTIPLE_LAUNCHES'),
    #                      "wrong status code for multiple launch files, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('MULTIPLE_LAUNCHES'), response.status.error_msg))
    #     self.assertEqual(len(response.path), 2, "wrong count of multiple launch files, result: %d, expected: %d" % (
    #         len(response.path), 2))
    #     path = xml.interpret_path(
    #         "$(find fkie_mas_daemon)/tests/resources/description_example.launch")
    #     response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_mas_daemon',
    #                                                     launch='description_example.launch', path=path, request_args=request_args), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('PARAMS_REQUIRED'),
    #                      "wrong status code for request arguments, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('PARAMS_REQUIRED'), response.status.error_msg))
    #     args = {arg.name: arg.value for arg in response.args}
    #     request_args = False
    #     response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_mas_daemon',
    #                                                     launch='description_example.launch', path=path, request_args=request_args), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'),
    #                      "wrong status code on successful load, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'), response.status.error_msg))

    #     response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_mas_daemon',
    #                                                     launch='description_example.launch', path=path, request_args=request_args), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('ALREADY_OPEN'),
    #                      "wrong status code if file is already open, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('ALREADY_OPEN'), response.status.error_msg))
    #     self.assertEqual(response.path[0], path, "Wrong returned path in ALREADY_OPEN reply, expected: %s, got: %s" % (
    #         response.path[0], path))

    #     # test loaded file
    #     response_stream = ls.GetLoadedFiles(lmsg.Empty(), DummyContext())
    #     items = [response for response in response_stream]
    #     self.assertEqual(
    #         len(items), 1, "The count of loaded files after successful load is wrong")
    #     self.assertEqual(items[0].path, path, "Wrong reported loaded file path, got: %s, expected: %s" % (
    #         items[0].path, path))
    #     self.assertEqual(items[0].package, 'fkie_mas_daemon', "Wrong reported loaded package, got: %s, expected: %s" % (
    #         items[0].package, 'fkie_mas_daemon'))
    #     self.assertEqual(items[0].launch, 'description_example.launch', "Wrong reported loaded launch, got: %s, expected: %s" % (
    #         items[0].launch, 'description_example.launch'))

    # def test_reload_file(self):
    #     ls = LaunchServicer(monitor_servicer=None, loop=None, test_env=True)
    #     path = xml.interpret_path(
    #         "$(find fkie_mas_daemon)/tests/resources/description_example.launch")
    #     response = ls.ReloadLaunch(lmsg.LaunchFile(path=path), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'),
    #                      "wrong status code on reload of not loaded file, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'), response.status.error_msg))
    #     response = ls.LoadLaunch(lmsg.LoadLaunchRequest(
    #         package='fkie_mas_daemon', launch='description_example.launch', path=path), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'),
    #                      "wrong status code on successful load, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'), response.status.error_msg))
    #     response = ls.ReloadLaunch(lmsg.LaunchFile(path=path), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'),
    #                      "wrong status code on reload of already loaded file, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'), response.status.error_msg))

    # def test_get_nodes(self):
    #     ls = LaunchServicer(monitor_servicer=None, loop=None, test_env=True)
    #     path = xml.interpret_path(
    #         "$(find fkie_mas_daemon)/tests/resources/description_example.launch")
    #     response = ls.LoadLaunch(lmsg.LoadLaunchRequest(
    #         package='fkie_mas_daemon', launch='description_example.launch', path=path), DummyContext())
    #     self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'),
    #                      "wrong status code on successful load, result: %d, expected: %d, reported error: %s"
    #                      % (response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'), response.status.error_msg))
    #     response_stream = ls.GetNodes(lmsg.ListNodesRequest(
    #         request_description=True), DummyContext())
    #     nodes = {}
    #     descriptions = []
    #     for response in response_stream:
    #         nodes[response.launch_file] = list(response.node)
    #         for descr in response.description:
    #             rd = RobotDescription(machine=descr.machine, robot_name=descr.robot_name, robot_type=descr.robot_type, robot_images=list(
    #                 descr.robot_images), robot_descr=descr.robot_descr)
    #             for cap in descr.capabilities:
    #                 cp = Capability(name=cap.name, namespace=cap.namespace, cap_type=cap.type, images=[
    #                                 img for img in cap.images], description=cap.description, nodes=[n for n in cap.nodes])
    #                 rd.capabilities.append(cp)
    #             descriptions.append(rd)
    #     self.assertEqual(len(
    #         nodes), 1, "wrong count of returned launch files for nodes, result: %d, expected: %d" % (len(nodes), 1))
    #     self.assertEqual(len(nodes[path]), 15, "wrong count of returned nodes, result: %d, expected: %d" % (
    #         len(nodes[path]), 15))
    #     self.assertEqual(len(descriptions), 2, "wrong count of descriptions, result: %d, expected: %d" % (
    #         len(descriptions), 2))
    #     self.assertEqual(descriptions[0].robot_name, 'pc', "wrong robot_name in first description, result: %s, expected: %s, description: %s" % (
    #         descriptions[0].robot_name, 'pc', descriptions[0]))
    #     self.assertEqual(len(descriptions[0].capabilities), 0, "wrong count of capabilities in first description, result: %d, expected: %d, description: %s" % (
    #         len(descriptions[0].capabilities), 0, descriptions[0]))
    #     self.assertEqual(len(descriptions[1].capabilities), 9, "wrong count of capabilities in second description, result: %d, expected: %d, description: %s" % (
    #         len(descriptions[1].capabilities), 9, descriptions[1]))

#         launch_manager.test_start_node('/example/test_node')


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestLaunchServicer)
