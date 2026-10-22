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

# cho_task_manager/behaviors/service/chunk_stream_completion_waiter.py
from std_srvs.srv import Trigger
from cho_task_manager.behaviors.service.base_service_server_behavior import BaseServiceServerBehavior
from cho_task_manager.utils.controller_names import chunk_stream_completion_service_name


class ChunkStreamCompletionWaiterBehavior(BaseServiceServerBehavior):
    # timeout_sec defaults to None (wait indefinitely): a timeout here returns FAILURE,
    # which shuts the tree down while action_chunk_controller keeps driving the robot -- there is
    # no failure-handling branch that cancels the chunk-stream goal or switches controllers, so a
    # bounded wait is only safe for trees that add one. Long policy rollouts (several
    # minutes) are legitimate, so an arbitrary default deadline would also cut them off.
    # `controller` selects which action-chunk controller's completion service to wait on, and
    # it is required. It must match the controller actually driving: the controller
    # derives the service from its own action name, and those differ per robot
    # (Franka `action_chunk_controller`, OpenArm MIT `action_chunk_mit_controller`). Pass
    # load_robot_config(...)['action_chunk'].
    #
    # `success_service` is the operator-facing "the task succeeded" trigger.
    # Franka's controller exposes it globally at /chunk_stream/trigger_success; the OpenArm
    # MIT controller exposes it controller-scoped, matching the rest of that
    # vertical's services, so its name has to be passed in.
    #
    # A completion that arrives while this leaf is not waiting is answered
    # success=false and does NOT trigger the success service: see
    # BaseServiceServerBehavior.
    def __init__(self, name="Wait_For_External_VLA_Script", timeout_sec: float = None,
                 controller=None, success_service: str = "/chunk_stream/trigger_success"):
        super().__init__(name, Trigger, chunk_stream_completion_service_name(controller),
                         timeout_sec=timeout_sec)
        self.response_message = "Task Manager acknowledged chunk-stream completion."
        self.success_service = success_service
        self.trigger_success_client = None

    def setup(self, **kwargs):
        result = super().setup(**kwargs)
        self.trigger_success_client = self.node.create_client(Trigger, self.success_service)
        return result

    def fill_response(self, request, response):
        if self.trigger_success_client.service_is_ready():
            self.trigger_success_client.call_async(Trigger.Request())
            self.node.get_logger().info(
                f"[{self.name}] Requested action-chunk controller reset via {self.success_service}.")
        else:
            self.node.get_logger().warn(
                f"[{self.name}] {self.success_service} is not available; "
                "controller may stay active.")

        return super().fill_response(request, response)
