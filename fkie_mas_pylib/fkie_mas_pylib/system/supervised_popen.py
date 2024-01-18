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

import subprocess
import threading
from fkie_mas_pylib.logging.logging import Log


class SupervisedPopen():
    '''
    The class overrides the subprocess.Popen and waits in a thread for its finish.
    If an error is printed out.
    '''

    def __init__(self, args, bufsize=0, executable=None, stdin=None, stdout=None,
                 stderr=subprocess.PIPE, preexec_fn=None, close_fds=False,
                 shell=False, cwd=None, env=None, universal_newlines=False,
                 startupinfo=None, creationflags=0, object_id='', description=''):
        '''
        For arguments see https://docs.python.org/2/library/subprocess.html
        Additional arguments:

        :param str object_id: the identification string of this object and title of the error message dialog
        :param str description: the description string used as additional information in dialog if an error was occurred
        '''
        self._args = args
        self._object_id = object_id
        Log.debug("start job [%s]" % self._object_id)
        self._description = description
        # wait for process to avoid 'defunct' processes
        self.popen = subprocess.Popen(args=args, bufsize=bufsize, executable=executable, stdin=stdin, stdout=stdout,
                                      stderr=stderr, preexec_fn=preexec_fn, close_fds=close_fds, shell=shell, cwd=cwd, env=env,
                                      universal_newlines=universal_newlines, startupinfo=startupinfo, creationflags=creationflags)
        thread = threading.Thread(target=self._supervise)
        thread.setDaemon(True)
        thread.start()

#   def __del__(self):
#     print "Deleted:", self._description

    @property
    def stdout(self):
        return self.popen.stdout

    @property
    def stderr(self):
        return self.popen.stderr

    @property
    def stdin(self):
        return self.popen.stdin

    def _supervise(self):
        '''
        Wait for process to avoid 'defunct' processes
        '''
        self.popen.wait()
        result_err = ''
        if self.stderr is not None:
            result_err = self.stderr.read()
        if result_err:
            Log.warn('%s - %s: %s' %
                     (self._object_id, self._description, result_err))
        Log.debug("job [%s] finished" % self._object_id)
