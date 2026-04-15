# cho_task_manager/behaviors/service/base_service_server_behavior.py
import py_trees
from rclpy.node import Node

class BaseServiceServerBehavior(py_trees.behaviour.Behaviour):
    """외부에서 서비스 호출(Signal)이 들어올 때까지 대기(RUNNING)하는 기본 서버 클래스"""
    def __init__(self, name: str, service_type, service_name: str):
        super().__init__(name)
        self.service_type = service_type
        self.service_name = service_name

        self.server = None
        self.node: Node = None
        self.signal_received = False
        self.response_message = "Signal received successfully."

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.server = self.node.create_service(
            self.service_type,
            self.service_name, 
            self._service_callback
        )
        self.node.get_logger().info(f"[{self.name}] Service Server Opened: {self.service_name}")
        return True

    def _service_callback(self, request, response):
        """클라이언트가 호출했을 때 실행되는 콜백"""
        self.signal_received = True
        self.node.get_logger().info(f"[{self.name}] Signal Received!")
        return self.fill_response(request, response)

    def fill_response(self, request, response):
        """자식 클래스에서 오버라이딩하여 Response 값을 채워넣는 함수"""
        # std_srvs/Trigger의 기본형태 (다른 타입이면 자식에서 덮어쓰기)
        if hasattr(response, 'success'):
            response.success = True
        if hasattr(response, 'message'):
            response.message = self.response_message
        return response

    def initialise(self):
        """노드 진입 시 매번 플래그 초기화"""
        self.signal_received = False
        self.node.get_logger().info(f"[{self.name}] Waiting for signal on {self.service_name}...")

    def update(self):
        """신호가 오면 SUCCESS, 안 오면 RUNNING"""
        if self.signal_received:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.signal_received = False