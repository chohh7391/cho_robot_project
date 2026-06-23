import py_trees
from std_srvs.srv import Trigger
from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior


class TareFTSensorServiceBehavior(BaseServiceBehavior):
    """Bota FT sensor의 tare(영점 조정) 서비스를 호출하는 behavior.

    bota tare는 서버가 ~3초간 샘플을 모아 영점을 계산한 뒤에야 응답을 보낸다.
    discovery server 환경에서는 그 응답이 유실되어 future가 끝나지 않는 경우가
    있으므로, wait_for_response=False면 요청만 보내고 바로 SUCCESS를 반환한다.
    (영점이 안정화될 시간은 뒤에 Timer 등으로 따로 확보한다.)
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

        # fire-and-forget: 요청을 보냈으면(future 생성됨) 응답을 기다리지 않고 통과
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
        """Trigger의 결과(result.success) 판별 (wait_for_response=True일 때만 사용)"""
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
