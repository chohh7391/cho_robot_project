import py_trees
from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior
from controller_manager_msgs.srv import ListControllers
from cho_task_manager.utils.controller_names import (
    LIST_CONTROLLERS_SERVICE,
    controller_name_value,
)


class ListControllersServiceBehavior(BaseServiceBehavior):
    def __init__(self, name: str, log_result: bool = False, require_active: list = None):
        super().__init__(name, ListControllers, LIST_CONTROLLERS_SERVICE)
        self.log_result = log_result
        # Controllers that must be in state "active" for this behavior to
        # SUCCEED (e.g. verifying a commandless controller survived a hold).
        self.require_active = [
            controller_name_value(c) for c in (require_active or [])
        ]

    def make_request(self):
        return ListControllers.Request()

    def handle_response(self, result):
        # The ListControllers response field is `controller` (controller_manager_msgs).
        states = {c.name: c.state for c in result.controller}

        if self.log_result:
            self.node.get_logger().info(f"[{self.name}] --- Current Controllers State ---")
            for controller_name, state in states.items():
                message = f"{controller_name}: {state}"
                if state == "active":
                    self.node.get_logger().info(message)
                else:
                    self.node.get_logger().debug(message)
            self.node.get_logger().info("-" * 40)

        missing = [
            name for name in self.require_active if states.get(name) != "active"
        ]
        if missing:
            self.node.get_logger().error(
                f"[{self.name}] Required controllers not active: {missing}"
            )
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
