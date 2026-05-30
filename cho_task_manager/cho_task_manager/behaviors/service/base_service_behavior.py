import py_trees
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class BaseServiceBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, service_type, service_name: str, timeout_sec: float = 3.0):
        super().__init__(name)
        self.service_type = service_type
        self.service_name = service_name
        self.timeout_sec = timeout_sec

        self.client = None
        self.node: Node = None
        self.future = None
        self.cb_group = None
        self.server_available = False

    def setup(self, **kwargs):
        """Service Client 공통 셋업"""
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
        """자식 클래스에서 오버라이딩하여 요청 메시지를 생성하는 함수"""
        return self.service_type.Request()

    def initialise(self):
        req = self.make_request()
        self.send_service_request(req)

    def send_service_request(self, req):
        """비동기 서비스 요청 함수"""
        if not self.client.wait_for_service(timeout_sec=0.1):
            self.node.get_logger().error(
                f"[{self.name}] Service server not available: {self.service_name}"
            )
            self.server_available = False
            self.future = None
            return

        self.server_available = True
        self.node.get_logger().info(f"[{self.name}] Sending Service Request...")
        self.future = self.client.call_async(req)

    def update(self):
        """응답이 올 때까지 RUNNING, 완료되면 응답을 공통 처리"""
        if self.future is None:
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
        """자식 클래스에서 오버라이딩하여 응답 데이터를 분석할 함수"""
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.future = None
