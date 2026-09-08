import py_trees
from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
from cho_task_manager.utils.controller_names import (
    SWITCH_CONTROLLER_SERVICE,
    controller_name_value,
    exclusive_arm_controllers,
)


class SwitchControllerServiceBehavior(BaseServiceBehavior):
    def __init__(
        self,
        name: str,
        activate: list,
        deactivate: list = None,
        exclusive: bool = True,
        strict: bool = None,
        activate_asap: bool = True,
        timeout_sec: int = 2,
        robot_config: dict = None,
        exclusive_controllers: list = None,
    ):
        """
        Switch controllers, by default exclusively.

        ``robot_config`` (the dict a tree builder receives) makes the exclusive
        set the one this robot actually has, taken from the canonical robot
        registry. Without it the historical Franka-only set is used, which is
        wrong for every other robot; pass it from any non-Franka tree.
        ``exclusive_controllers`` overrides the set outright.
        """
        super().__init__(name, SwitchController, SWITCH_CONTROLLER_SERVICE)
        self.activate = activate

        if exclusive:
            # Deactivate every mutually-exclusive arm controller except the one(s)
            # being activated. This makes the switch idempotent: it succeeds no
            # matter which controller was active before (e.g. when the mission
            # sequence re-runs from the top after a mid-sequence failure), instead
            # of assuming a fixed predecessor via a hard-coded deactivate list.
            candidates = (
                exclusive_controllers if exclusive_controllers is not None
                else exclusive_arm_controllers(robot_config)
            )
            keep = {controller_name_value(c) for c in activate}
            self.deactivate = [
                c for c in candidates
                if controller_name_value(c) not in keep
            ]
            # BEST_EFFORT so deactivating a controller that is not currently
            # running is tolerated (STRICT would fail the whole switch).
            self.strict = False if strict is None else strict
        else:
            self.deactivate = deactivate or []
            self.strict = True if strict is None else strict

        self.activate_asap = activate_asap
        self.timeout_sec = timeout_sec

    def make_request(self):
        req = SwitchController.Request()
        req.activate_controllers = [
            controller_name_value(controller) for controller in self.activate
        ]
        req.deactivate_controllers = [
            controller_name_value(controller) for controller in self.deactivate
        ]
        req.strictness = (
            SwitchController.Request.STRICT
            if self.strict
            else SwitchController.Request.BEST_EFFORT
        )
        req.activate_asap = self.activate_asap
        req.timeout = Duration(sec=self.timeout_sec, nanosec=0)
        return req

    def handle_response(self, result):
        """Judge the SwitchController result (result.ok)."""
        if result.ok:
            self.node.get_logger().info(f"[{self.name}] Controllers Switched Successfully!")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error(
                f"[{self.name}] Failed to switch controllers. "
                f"activate={self.activate}, deactivate={self.deactivate}"
            )
            return py_trees.common.Status.FAILURE
