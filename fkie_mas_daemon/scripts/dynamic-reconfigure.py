#!/usr/bin/env python3

import sys

try:
    # renamed in commit #973d0d8 -> https://github.com/ros-visualization/rqt_reconfigure/commit/973d0d8bf614e27fdb6ce12aaaf8e65dce348ff1#diff-3809107d101d214d37251fe0bff88d74
    from rqt_reconfigure.param_client_widget import ParamClientWidget as DynreconfClientWidget
except ImportError:
    from rqt_reconfigure.dynreconf_client_widget import DynreconfClientWidget
import dynamic_reconfigure.client
import rospy


app = None


def setTerminalName(name):
    '''
    Change the terminal name.

    :param str name: New name of the terminal
    '''
    sys.stdout.write("".join(["\x1b]2;", name, "\x07"]))


def setProcessName(name):
    '''
    Change the process name.

    :param str name: New process name
    '''
    try:
        from ctypes import cdll, byref, create_string_buffer
        libc = cdll.LoadLibrary('libc.so.6')
        buff = create_string_buffer(len(name) + 1)
        buff.value = name
        libc.prctl(15, byref(buff), 0, 0, 0)
    except:
        pass


def finish(*arg):
    '''
    Callback called on exit of the ros node.
    '''
    # close all ssh sessions
    global app
    if not app is None:
        app.exit()


def main(argv=sys.argv):
    args = rospy.myargv(argv=sys.argv)
    node = args[1]
    try:
        from python_qt_binding.QtGui import QApplication, QScrollArea
    except:
        try:
            from python_qt_binding.QtWidgets import QApplication, QScrollArea
        except:
            sys.stderr.write("please install 'python_qt_binding' package!!")
            sys.exit(-1)
    rospy.init_node(node.replace(rospy.names.SEP, '_').strip(
        '_'), log_level=rospy.DEBUG)
    setTerminalName(rospy.get_name())
    setProcessName(rospy.get_name())

    # Initialize Qt
    global app
    app = QApplication(args)

    try:
        dynreconf_client = dynamic_reconfigure.client.Client(
            str(node), timeout=5.0)
    except rospy.exceptions.ROSException:
        rospy.logerr("Could not connect to %s" % node)
        # TODO(Isaac) Needs to show err msg on GUI too.
        return

    _scroll_area = QScrollArea()
    _dynreconf_client = DynreconfClientWidget(dynreconf_client, node)
    _scroll_area.resize(_dynreconf_client.width(
    ) + 30, 480 if _dynreconf_client.height() > 480 else _dynreconf_client.height())
    _scroll_area.setWidget(_dynreconf_client)
    _scroll_area.show()
    exit_code = -1
    rospy.on_shutdown(finish)
    exit_code = app.exec_()


if __name__ == '__main__':
    main()
