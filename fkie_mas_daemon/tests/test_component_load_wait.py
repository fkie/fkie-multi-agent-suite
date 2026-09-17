# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Michael Swan
# License: MIT
#
# ****************************************************************************

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from fkie_mas_daemon.ros_node import RosNodeLauncher

from fkie_mas_daemon.launch import launch_config


class TestComponentLoadWait(unittest.TestCase):
    def test_call_service_without_deadline_polls_until_available(self):
        response = object()
        future = Mock()
        future.exception.return_value = None
        future.result.return_value = response
        future.add_done_callback.side_effect = lambda callback: callback(future)

        client = Mock()
        client.wait_for_service.side_effect = [False, True]
        client.call_async.return_value = future

        launcher = object.__new__(RosNodeLauncher)
        launcher._on_shutdown = False
        launcher.ros_node = Mock()
        launcher.ros_node.create_client.return_value = client

        with patch("fkie_mas_daemon.ros_node.rclpy.ok", return_value=True):
            result = launcher.call_service("/container/_container/load_node", Mock, Mock(), timeout_sec=None)

        self.assertIs(result, response)
        self.assertEqual(client.wait_for_service.call_count, 2)
        launcher.ros_node.destroy_client.assert_called_once_with(client)

    def test_composed_node_uses_component_timeout(self):
        request = SimpleNamespace(node_name="/component", plugin_name="test::Component")
        response = SimpleNamespace(success=True, full_node_name="/component")
        launcher = Mock()
        launcher.call_service.return_value = response
        node = SimpleNamespace(
            composable_container="/container",
            unique_name="/component",
            get_composed_load_request=lambda: request,
        )

        with patch.object(launch_config.nmd, "launcher", launcher):
            launch_config.LaunchConfig.run_composed_node(object(), node)

        self.assertEqual(launcher.call_service.call_args.kwargs["timeout_sec"], 30.0)


if __name__ == "__main__":
    unittest.main()
