# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import py_trees
from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
from cho_task_manager.utils.controller_names import (
    SWITCH_CONTROLLER_SERVICE,
    controller_name_value,
    exclusive_arm_controllers,
)


def _duration(seconds):
    """A builtin_interfaces Duration for a float number of seconds.

    ``Duration(sec=...)`` takes an int and raises on a float, so the seconds
    are split here rather than passed through.
    """
    whole = int(math.floor(seconds))
    nanos = int(round((seconds - whole) * 1e9))
    if nanos >= 1_000_000_000:          # rounding can carry
        whole += 1
        nanos -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanos)


class SwitchControllerServiceBehavior(BaseServiceBehavior):
    def __init__(
        self,
        name: str,
        activate: list,
        deactivate: list = None,
        exclusive: bool = True,
        strict: bool = None,
        activate_asap: bool = True,
        switch_timeout_sec: float = 2.0,
        robot_config: dict = None,
        exclusive_controllers: list = None,
    ):
        """
        Switch controllers, by default exclusively.

        An exclusive switch needs ``robot_config`` (the dict a tree builder
        receives): the set it deactivates is the one this robot actually has,
        taken from the canonical robot registry. ``exclusive_controllers``
        overrides the set outright. With neither it raises -- there is no
        robot-independent set to fall back to.

        ``switch_timeout_sec`` is the controller_manager's own deadline for the
        switch (the request's ``timeout``). It is deliberately not called
        ``timeout_sec``: that is the base class's service-discovery wait, and
        the two used to share one attribute.
        """
        super().__init__(name, SwitchController, SWITCH_CONTROLLER_SERVICE)
        if (isinstance(switch_timeout_sec, bool)
                or not isinstance(switch_timeout_sec, (int, float))
                or not math.isfinite(switch_timeout_sec) or switch_timeout_sec < 0.0):
            raise ValueError(
                f'[{name}] switch_timeout_sec must be a finite, non-negative number of '
                f'seconds; got {switch_timeout_sec!r}')
        self.activate = activate

        if exclusive:
            # Deactivate every mutually-exclusive arm controller except the one(s)
            # being activated. This makes the switch idempotent: it succeeds no
            # matter which controller was active before (e.g. when the mission
            # sequence re-runs from the top after a mid-sequence failure), instead
            # of assuming a fixed predecessor via a hard-coded deactivate list.
            if exclusive_controllers is None and robot_config is None:
                raise ValueError(
                    f'[{name}] an exclusive switch needs robot_config (or '
                    'exclusive_controllers): which controllers hold the arm is '
                    "the robot's, and deactivating some other robot's set would "
                    'leave this one\'s running')
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
        self.switch_timeout_sec = float(switch_timeout_sec)

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
        req.timeout = _duration(self.switch_timeout_sec)
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
