import py_trees
from std_srvs.srv import Trigger
from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior


class TareFTSensorServiceBehavior(BaseServiceBehavior):
    """
    Calls the Bota FT sensor's tare (zeroing) service.

    The bota tare server only responds after collecting samples for ~3s to compute the
    zero offset. In a discovery-server setup that response can get lost, leaving the
    future never done -- so wait_for_response=False just sends the request and returns
    SUCCESS immediately (give the zero time to settle separately, e.g. with a Timer).
    """

    def __init__(
        self,
        name: str = "Tare_FT_Sensor",
        service_name: str = "/bota_ft_sensor/tare",
        timeout_sec: float = 3.0,
        wait_for_response: bool = False,
    ):
        super().__init__(name, Trigger, service_name, timeout_sec=timeout_sec)
        self.wait_for_response = wait_for_response

    def update(self):
        if self.wait_for_response:
            return super().update()

        # fire-and-forget: pass as soon as the request was sent (future exists),
        # without waiting for a response
        if self.future is None:
            self.node.get_logger().error(
                f"[{self.name}] Tare request was not sent (service unavailable)."
            )
            return py_trees.common.Status.FAILURE

        self.node.get_logger().info(
            f"[{self.name}] Tare request sent (not waiting for response)."
        )
        return py_trees.common.Status.SUCCESS

    def handle_response(self, result):
        """Interprets the Trigger result (only used when wait_for_response=True)."""
        if result.success:
            self.node.get_logger().info(
                f"[{self.name}] FT sensor tared successfully! message='{result.message}'"
            )
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error(
                f"[{self.name}] Failed to tare FT sensor. message='{result.message}'"
            )
            return py_trees.common.Status.FAILURE
