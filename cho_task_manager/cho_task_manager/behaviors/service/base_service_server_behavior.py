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

import py_trees
from rclpy.node import Node

from cho_task_manager.utils.clock import arm, deadline_after


class BaseServiceServerBehavior(py_trees.behaviour.Behaviour):
    """Base server behavior: RUNNING until an external service call (signal) arrives.

    The server exists from setup() for the whole run, but a signal is only
    ACCEPTED while this leaf is waiting for one -- between initialise() and
    terminate(). A call at any other time is answered ``success=false`` with
    the reason, and nothing else happens: telling the caller its signal was
    acknowledged when no step of the tree was listening is how a completion
    gets reported as handled and then silently lost.
    """

    def __init__(self, name: str, service_type, service_name: str, timeout_sec: float = None):
        super().__init__(name)
        self.service_type = service_type
        self.service_name = service_name
        self.timeout_sec = timeout_sec

        self.server = None
        self.node: Node = None
        self.signal_received = False
        self.waiting = False
        self.response_message = "Signal received successfully."
        self._deadline = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.server = self.node.create_service(
            self.service_type,
            self.service_name,
            self._service_callback
        )
        self.node.get_logger().info(f"[{self.name}] Service Server Opened: {self.service_name}")
        return True

    def _service_callback(self, request, response):
        if not self.waiting:
            message = (f"{self.name} is not waiting for a signal on {self.service_name}; "
                       "nothing was done with this one.")
            self.node.get_logger().warn(f"[{self.name}] Signal refused: {message}")
            return self.reject_response(request, response, message)
        self.signal_received = True
        self.node.get_logger().info(f"[{self.name}] Signal received.")
        return self.fill_response(request, response)

    def fill_response(self, request, response):
        """Override in a subclass to fill in the response. Default matches std_srvs/Trigger."""
        if hasattr(response, 'success'):
            response.success = True
        if hasattr(response, 'message'):
            response.message = self.response_message
        return response

    def reject_response(self, request, response, message):
        """The answer to a signal that arrived while nothing was waiting for it."""
        if hasattr(response, 'success'):
            response.success = False
        if hasattr(response, 'message'):
            response.message = message
        return response

    def initialise(self):
        self.signal_received = False
        # None while the node clock reads 0 (utils/clock.py); update() takes it then.
        self._deadline = None
        if self.timeout_sec is not None:
            self._deadline = deadline_after(self.node.get_clock(), self.timeout_sec)
        self.waiting = True
        self.node.get_logger().info(f"[{self.name}] Waiting for signal on {self.service_name}...")

    def update(self):
        if self.signal_received:
            return py_trees.common.Status.SUCCESS
        if self._timed_out():
            self.node.get_logger().error(
                f"[{self.name}] Timed out after {self.timeout_sec}s waiting for signal on "
                f"{self.service_name}"
            )
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def _timed_out(self):
        if self.timeout_sec is None:
            return False
        clock = self.node.get_clock()
        self._deadline = arm(self._deadline, clock, self.timeout_sec)
        return self._deadline is not None and clock.now() > self._deadline

    def terminate(self, new_status):
        self.waiting = False
        self.signal_received = False
        self._deadline = None
