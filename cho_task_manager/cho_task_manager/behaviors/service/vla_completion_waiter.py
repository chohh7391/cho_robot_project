# cho_task_manager/behaviors/wait/vla_wait_behavior.py
from std_srvs.srv import Trigger
from cho_task_manager.behaviors.service.base_service_server_behavior import BaseServiceServerBehavior
from cho_task_manager.utils.controller_names import vla_completion_service_name

class VLACompletionWaiterBehavior(BaseServiceServerBehavior):
    def __init__(self, name="Wait_For_External_VLA_Script"):
        super().__init__(name, Trigger, vla_completion_service_name())
        self.response_message = "Task Manager acknowledged VLA completion."
