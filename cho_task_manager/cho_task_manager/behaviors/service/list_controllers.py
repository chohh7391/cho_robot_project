import py_trees
from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior
from controller_manager_msgs.srv import ListControllers


class ListControllersServiceBehavior(BaseServiceBehavior):
    def __init__(self, name: str):
        super().__init__(name, ListControllers, '/controller_manager/list_controllers')

    def initialise(self):
        req = ListControllers.Request()
        self.send_service_request(req)

    def process_response(self, result):
        """현재 켜져 있는 제어기 목록을 터미널에 예쁘게 출력"""
        self.node.get_logger().info(f"[{self.name}] --- Current Controllers State ---")
        
        for controller in result.controller_state:
            state_str = f"{controller.name}: {controller.state}"
            if controller.state == "active":
                self.node.get_logger().info(state_str) # 초록색 등 강조 가능
            else:
                self.node.get_logger().warn(state_str)
                
        self.node.get_logger().info("-" * 40)
        return py_trees.common.Status.SUCCESS