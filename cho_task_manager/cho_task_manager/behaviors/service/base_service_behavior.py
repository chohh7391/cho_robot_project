import py_trees
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class BaseServiceBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, service_type, service_name: str):
        super().__init__(name)
        self.service_type = service_type
        self.service_name = service_name

        self.client = None
        self.node: Node = None
        self.future = None
        self.cb_group = None

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
        self.client.wait_for_service(timeout_sec=3.0)

    def send_service_request(self, req):
        """자식 클래스의 initialise()에서 호출할 비동기 요청 함수"""
        self.node.get_logger().info(f"[{self.name}] Sending Service Request...")
        self.future = self.client.call_async(req)

    def update(self):
        """10Hz마다 응답이 왔는지 체크 (RUNNING 유지)"""
        if self.future is None:
            return py_trees.common.Status.FAILURE

        # 1. 서버가 아직 처리 중이면 트리 진행을 멈추고 대기
        if not self.future.done():
            return py_trees.common.Status.RUNNING

        # 2. 응답 도착!
        result = self.future.result()
        if result is not None:
            # 응답 처리는 자식 클래스의 process_response()에 위임!
            return self.process_response(result)
        else:
            self.node.get_logger().error(f"[{self.name}] Service call failed entirely!")
            return py_trees.common.Status.FAILURE

    def process_response(self, result):
        """자식 클래스에서 오버라이딩하여 응답 데이터를 분석할 함수"""
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.future = None