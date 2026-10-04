"""FT sensor preparation shared by the contact-rich forge tasks."""

from cho_task_manager.behaviors.service import TareFTSensorServiceBehavior
from cho_task_manager.behaviors.wait import WaitBehavior

# The Bota driver keeps publishing through the tare; the bias only settles a
# few hundred ms later, and a task that reads force before then reads the
# pre-tare offset. Three seconds is the value the forge tasks were tuned with,
# on the node's clock: in simulation the bias settles in sim time.
DEFAULT_SETTLE_SEC = 3.0


def tare_ft_children(settle_sec=DEFAULT_SETTLE_SEC):
    """Zero the FT sensor and wait for the bias to settle.

    Returns the two behaviours rather than a composite: callers splice them
    into the front of their own initialise sequence, which is where the names
    ``Tare_FT_Sensor`` / ``Wait_After_Tare`` are asserted from.
    """
    return [
        TareFTSensorServiceBehavior(name='Tare_FT_Sensor'),
        WaitBehavior(name='Wait_After_Tare', duration_sec=settle_sec),
    ]
