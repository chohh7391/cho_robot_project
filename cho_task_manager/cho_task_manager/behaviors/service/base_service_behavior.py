import py_trees
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class BaseServiceBehavior(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        service_type,
        service_name: str,
        timeout_sec: float = 3.0,
        response_timeout_sec: float = 30.0,
    ):
        super().__init__(name)
        self.service_type = service_type
        self.service_name = service_name
        self.timeout_sec = timeout_sec
        self.response_timeout_sec = response_timeout_sec

        self.client = None
        self.node: Node = None
        self.future = None
        self.cb_group = None
        self.server_available = False
        self._deadline = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cb_group = ReentrantCallbackGroup()
        self.client = self.node.create_client(
            self.service_type, 
            self.service_name, 
            callback_group=self.cb_group
        )
        self.node.get_logger().info(f"[{self.name}] Waiting for {self.service_name} Server...")
        self.server_available = self.client.wait_for_service(timeout_sec=self.timeout_sec)
        if not self.server_available:
            self.node.get_logger().warn(
                f"[{self.name}] Service server not available during setup: {self.service_name}"
            )
        return True

    def make_request(self):
        """Override in a subclass to build the request message."""
        return self.service_type.Request()

    def initialise(self):
        req = self.make_request()
        self.send_service_request(req)

    def send_service_request(self, req):
        if not self.client.wait_for_service(timeout_sec=0.1):
            self.node.get_logger().error(
                f"[{self.name}] Service server not available: {self.service_name}"
            )
            self.server_available = False
            self.future = None
            self._deadline = None
            return

        self.server_available = True
        self.node.get_logger().info(f"[{self.name}] Sending Service Request...")
        self.future = self.client.call_async(req)
        self._deadline = self.node.get_clock().now() + Duration(seconds=self.response_timeout_sec)

    def _timed_out(self):
        return self._deadline is not None and self.node.get_clock().now() > self._deadline

    def update(self):
        if self.future is None:
            return py_trees.common.Status.FAILURE

        if self._timed_out():
            self.node.get_logger().error(
                f"[{self.name}] Timed out after {self.response_timeout_sec}s waiting for "
                f"{self.service_name} response"
            )
            # Untrack the abandoned request in the rclpy Client -- otherwise each
            # timed-out attempt leaks an entry in its pending-request map forever.
            self.client.remove_pending_request(self.future)
            self.future = None
            self._deadline = None
            return py_trees.common.Status.FAILURE

        if not self.future.done():
            return py_trees.common.Status.RUNNING

        try:
            result = self.future.result()
        except Exception as exc:
            self.node.get_logger().error(f"[{self.name}] Service call failed: {exc}")
            return py_trees.common.Status.FAILURE

        if result is None:
            self.node.get_logger().error(f"[{self.name}] Service call returned no response.")
            return py_trees.common.Status.FAILURE

        return self.handle_response(result)

    def handle_response(self, result):
        """Override in a subclass to interpret the response."""
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if self.future is not None and not self.future.done():
            # Preempted with the request still in flight: untrack it (see update()).
            self.client.remove_pending_request(self.future)
        self.future = None
        self._deadline = None
