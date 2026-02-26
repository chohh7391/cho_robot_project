import py_trees
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from cho_task_manager.utils.controller_names import ControllerNames


class BaseActionBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, action_type, action_name: str):
        super().__init__(name)

        valid_controllers = [f"/controller_action_server/{c.value}" for c in ControllerNames]
        assert str(action_name) in valid_controllers, \
            f"Invalid controller name: '{action_name}'. Must be one of {valid_controllers}"
        
        self.action_type = action_type
        self.action_name = action_name

        self.client = None
        self.node: Node = None
        self.send_goal_future = None
        self.get_result_future = None
        self.goal_handle = None
        self.cb_group = None

    def setup(self, **kwargs):
        """Action Client 공통 셋업"""
        self.node = kwargs['node']
        self.cb_group = ReentrantCallbackGroup()
        self.client = ActionClient(
            self.node, 
            self.action_type, 
            self.action_name, 
            callback_group=self.cb_group
        )
        self.node.get_logger().info(f"[{self.name}] Waiting for {self.action_name} Server...")
        self.client.wait_for_server(timeout_sec=3.0)

    def send_action_goal(self, goal_msg):
        """자식 클래스의 initialise()에서 호출할 핵심 헬퍼 함수"""
        self.node.get_logger().info(f"[{self.name}] Sending Goal...")
        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.get_result_future = None
        self.goal_handle = None

    def update(self):
        """비동기 상태 체크 (모든 Action에 100% 동일하게 적용)"""
        if self.send_goal_future is None:
            return py_trees.common.Status.FAILURE

        # 1. Goal 전송 중...
        if not self.send_goal_future.done():
            return py_trees.common.Status.RUNNING
        
        # 2. Goal 도착 및 수락 여부 확인
        if self.goal_handle is None:
            self.goal_handle = self.send_goal_future.result()
            if not self.goal_handle.accepted:
                self.node.get_logger().error(f"[{self.name}] Goal Rejected!")
                return py_trees.common.Status.FAILURE
            self.get_result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
            
        # 3. 로봇이 움직이는 중 (결과 대기)
        if not self.get_result_future.done():
            return py_trees.common.Status.RUNNING
            
        # 4. 동작 완료! 결과 판별
        result = self.get_result_future.result()
        if result.status == 4: # STATUS_SUCCEEDED
            self.node.get_logger().info(f"[{self.name}] Action Succeeded!")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error(f"[{self.name}] Action Failed with status: {result.status}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """공통 강제 취소(Preempt) 로직"""
        if new_status == py_trees.common.Status.INVALID and self.goal_handle is not None:
            if self.get_result_future is not None and not self.get_result_future.done():
                self.node.get_logger().warn(f"[{self.name}] Preempted! Canceling Goal...")
                self.goal_handle.cancel_goal_async()
        
        self.send_goal_future = None
        self.get_result_future = None
        self.goal_handle = None