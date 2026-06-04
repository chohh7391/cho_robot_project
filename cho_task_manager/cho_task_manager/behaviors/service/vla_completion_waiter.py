# cho_task_manager/behaviors/wait/vla_wait_behavior.py
from std_srvs.srv import Trigger
from cho_task_manager.behaviors.service.base_service_server_behavior import BaseServiceServerBehavior
from cho_task_manager.utils.controller_names import vla_completion_service_name

class VLACompletionWaiterBehavior(BaseServiceServerBehavior):
    def __init__(self, name="Wait_For_External_VLA_Script"):
        super().__init__(name, Trigger, vla_completion_service_name())
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