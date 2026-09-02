#!/usr/bin/env python3
"""Exit successfully only after the static planning scene gate reports ready."""

import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main(args=None):
    rclpy.init(args=args)
    node = Node('wait_for_fr5_static_planning_scene')
    node.declare_parameter('timeout', 30.0)
    node.declare_parameter('ready_service', '/static_scene_ready')
    deadline = time.monotonic() + float(node.get_parameter('timeout').value)
    client = node.create_client(Trigger, node.get_parameter('ready_service').value)
    success = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=0.5):
                continue
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            if future.done() and future.exception() is None and future.result().success:
                node.get_logger().info(future.result().message)
                success = True
                break
            time.sleep(0.2)
        if not success:
            node.get_logger().fatal('Timed out waiting for the static planning scene safety gate')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 3


if __name__ == '__main__':
    sys.exit(main())
