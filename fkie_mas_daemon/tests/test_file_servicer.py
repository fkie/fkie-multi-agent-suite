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

from io import FileIO
import os
import unittest
import time
import json
from types import SimpleNamespace

from fkie_mas_pylib.launch import xml
from fkie_mas_daemon.file_servicer import FileServicer

PKG = 'fkie_mas_daemon'


class TestFileServiceServicer(unittest.TestCase):
    '''
    '''

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.current_pose = 0
        path = xml.interpret_path(
            '$(find fkie_mas_daemon)/../../../build')
        if not os.path.exists(path):
            os.mkdir(path)
        self.test_get_content_path = '%s/tmp_get_content_test.txt' % path
        self.test_save_content_path = '%s/tmp_save_content_test.txt' % path
        self.test_rename_from_file = '%s/tmp_rename_from_dummy.launch' % path
        self.test_rename_to_file = '%s/tmp_rename_to_dummy.launch' % path

    def tearDown(self):
        try:
            os.remove(self.test_get_content_path)
        except Exception:
            pass
        try:
            os.remove(self.test_save_content_path)
        except Exception:
            pass
        try:
            os.remove(self.test_rename_from_file)
        except Exception:
            pass
        try:
            os.remove(self.test_rename_to_file)
        except Exception:
            pass

    def test_list_path(self):
        fs = FileServicer(loop=None, test_env=True)
        root_paths = set(os.getenv('ROS_PACKAGE_PATH').split(':'))
#        launch_response = fs.ListPath(fmsg.ListPathRequest(path=''), DummyContext())
#        self.assertEqual(len(root_paths), len(launch_response.items), 'ROS root paths are not equal, expected: %s, got: %s' % (root_paths, launch_response.items))
        launch_response = fs.getPathList(inputPath=os.getcwd())
        launch_response = json.loads(
            launch_response, object_hook=lambda d: SimpleNamespace(**d))
        self.assertEqual(len(os.listdir(os.getcwd())), len(launch_response),
                         'reported different count of items in working directory: %s' % os.getcwd())
        # # test cache
        # launch_response = fs.ListPath(fmsg.ListPathRequest(
        #     path='%s/../..' % os.getcwd()), DummyContext())
        # count_dirs = len(
        #     [d for d in launch_response.items if d.type in [1, 3]])
        # launch_response = fs.ListPath(fmsg.ListPathRequest(
        #     path='%s/../..' % os.getcwd()), DummyContext())
        # self.assertEqual(len([d for d in launch_response.items if d.type in [
        #                  1, 3]]), count_dirs, 'count of directories from cache is different')
        # # test invalid path
        # launch_response = fs.ListPath(
        #     fmsg.ListPathRequest(path='/unknown'), DummyContext())
        # self.assertEqual(0, len(launch_response.items),
        #                  'list of invalid path returns more then 0 items')
        # self.assertEqual(fmsg.ReturnStatus.StatusType.Value(
        #     'OS_ERROR'), launch_response.status.code, 'wrong status code if path not exists')

    def test_list_packages(self):
        fs = FileServicer(loop=None, test_env=True)
        packages_response = fs.getPackageList()

    def test_get_content(self):
        fs = FileServicer(loop=None, test_env=True)
        fs.FILE_CHUNK_SIZE = 10
        # content_response = fs.getFileContent(requestPath='')
        # self.assertEqual(next(content_response).status.code, fmsg.ReturnStatus.StatusType.Value(
        #     'IO_ERROR'), 'wrong status code if path is empty')
        # content_response = fs.GetFileContent(fmsg.GetFileContentRequest(
        #     path=self.test_get_content_path), DummyContext())
        # self.assertEqual(next(content_response).status.code, fmsg.ReturnStatus.StatusType.Value(
        #     'IO_ERROR'), 'wrong status code if path not exists')
        # create a test file
        test_data = 'This is a test file for get content test.'
        with FileIO(self.test_get_content_path, 'w') as testfile:
            testfile.write(test_data.encode())
        content_response = fs.getFileContent(
            requestPath=self.test_get_content_path)
        content_response = json.loads(
            content_response, object_hook=lambda d: SimpleNamespace(**d)
        )
        self.assertEqual(
            content_response.path, self.test_get_content_path, 'wrong returned path in file content')
        self.assertGreater(
            content_response.mtime, 0, 'wrong returned file mtime in file GetFileContentReply: %.1f, expected: >0' % content_response.mtime)
        self.assertEqual(content_response.size, len(
            test_data), 'wrong returned file size in file GetFileContentReply: %d, expected: %d' % (content_response.size, len(test_data)))
        self.assertEqual(len(content_response.value), len(
            test_data), 'wrong returned length of data in file GetFileContentReply: %d, expected: %d' % (len(content_response.value), len(test_data)))
        self.assertEqual(content_response.value, test_data, 'wrong returned data in file GetFileContentReply: %s, expected: %s' % (
            content_response.value, test_data))
        os.remove(self.test_get_content_path)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestFileServiceServicer)
