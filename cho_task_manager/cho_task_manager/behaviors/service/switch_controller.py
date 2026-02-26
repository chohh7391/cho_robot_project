import py_trees
from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration 


class SwitchControllerServiceBehavior(BaseServiceBehavior):
    def __init__(self, name: str, activate: list, deactivate: list):
        super().__init__(name, SwitchController, '/controller_manager/switch_controller')
        self.activate = activate
        self.deactivate = deactivate

    def initialise(self):
        req = SwitchController.Request()
        req.activate_controllers = self.activate
        req.deactivate_controllers = self.deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = Duration(sec=2, nanosec=0)

        self.send_service_request(req)

    def process_response(self, result):
        """SwitchController의 결과(result.ok) 판별"""
        if result.ok:
            self.node.get_logger().info(f"[{self.name}] Controllers Switched Successfully!")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to switch controllers!")
            return py_trees.common.Status.FAILURE