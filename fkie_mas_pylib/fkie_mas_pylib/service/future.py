import rclpy
from rclpy.client import SrvType
from rclpy.client import SrvTypeRequest
from rclpy.node import Node
import time
from typing import List


class WaitFuture:
    type: str
    node_name: str
    service_name: str
    future: rclpy.task.Future
    client: rclpy.client.Client
    finished: bool

    def __init__(self,
                 type: str,
                 node_name: str,
                 service_name: str,
                 future: rclpy.task.Future,
                 client: rclpy.client.Client):
        self.type = type
        self.node_name = node_name
        self.service_name = service_name
        self.future = future
        self.client = client
        self.finished = False
        future.add_done_callback(self.done_callback)

    def done_callback(self, future: rclpy.task.Future):
        self.finished = future.done()


def create_service_future(node: Node, wait_futures: List[WaitFuture], type: str, node_name: str, service_name: str, srv_type: SrvType, request: SrvTypeRequest) -> bool:
    client = node.create_client(srv_type, service_name)
    if client.service_is_ready():
        ros_future = client.call_async(request)
        wait_futures.append(WaitFuture(type, node_name, service_name, ros_future, client))
        return True
    else:
        client.destroy()
        return False


def wait_until_futures_done(futures: List[WaitFuture], timeout: float = 5.0):
    start_ts = time.time()

    def check_finished(futures):
        for future in futures:
            if not future.finished:
                return False
        return True

    while not check_finished(futures) and time.time() - start_ts < timeout:
        time.sleep(0.04)
    for future in futures:
        if not future.finished:
            future.future.cancel()
