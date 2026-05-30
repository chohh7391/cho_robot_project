import py_trees
from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior
from controller_manager_msgs.srv import ListControllers
from cho_task_manager.utils.controller_names import LIST_CONTROLLERS_SERVICE


class ListControllersServiceBehavior(BaseServiceBehavior):
    def __init__(self, name: str, log_result: bool = False):
        super().__init__(name, ListControllers, LIST_CONTROLLERS_SERVICE)
        self.log_result = log_result

    def make_request(self):
        return ListControllers.Request()

    def handle_response(self, result):
        """현재 controller state를 필요할 때만 출력"""
        if not self.log_result:
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(f"[{self.name}] --- Current Controllers State ---")
        
        for controller in result.controller_state:
            state_str = f"{controller.name}: {controller.state}"
            if controller.state == "active":
                self.node.get_logger().info(state_str)
            else:
                self.node.get_logger().debug(state_str)
                
        self.node.get_logger().info("-" * 40)
        return py_trees.common.Status.SUCCESS
