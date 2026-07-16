# cho_task_manager/behaviors/wait/vla_wait_behavior.py
from std_srvs.srv import Trigger
from cho_task_manager.behaviors.service.base_service_server_behavior import BaseServiceServerBehavior
from cho_task_manager.utils.controller_names import vla_completion_service_name

class VLACompletionWaiterBehavior(BaseServiceServerBehavior):
    # timeout_sec defaults to None (wait indefinitely): a timeout here returns FAILURE,
    # which shuts the tree down while vla_controller keeps driving the robot -- there is
    # no failure-handling branch that cancels the VLA goal or switches controllers, so a
    # bounded wait is only safe for trees that add one. Long VLA rollouts (several
    # minutes) are legitimate, so an arbitrary default deadline would also cut them off.
    def __init__(self, name="Wait_For_External_VLA_Script", timeout_sec: float = None):
        super().__init__(name, Trigger, vla_completion_service_name(), timeout_sec=timeout_sec)
        self.response_message = "Task Manager acknowledged VLA completion."
        self.trigger_success_client = None

    def setup(self, **kwargs):
        result = super().setup(**kwargs)
        self.trigger_success_client = self.node.create_client(Trigger, "/vla/trigger_success")
        return result

    def fill_response(self, request, response):
        if self.trigger_success_client.service_is_ready():
            self.trigger_success_client.call_async(Trigger.Request())
            self.node.get_logger().info(f"[{self.name}] Requested VLA controller reset via /vla/trigger_success.")
        else:
            self.node.get_logger().warn(f"[{self.name}] /vla/trigger_success is not available; controller may stay active.")

        return super().fill_response(request, response)