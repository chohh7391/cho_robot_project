"""Wait on the node's clock, which is the simulator's when use_sim_time is set.

``py_trees.timers.Timer`` measures WALL time. Every wait in these trees exists
for something physical to finish -- an FT bias to settle, jaws to stop, an arm
to come to rest -- and in simulation that happens in SIM time. A simulator
running slower than real time (Isaac at a fraction of real time, MuJoCo under
load) then gets less simulated settling than the wait was tuned for, and the
step after it reads a sensor or moves an arm that has not settled yet.

This leaf reads ``node.get_clock()``: sim time when ``use_sim_time`` is set,
wall time otherwise, so a wait means the same amount of physics either way.
"""

import math

import py_trees
from rclpy.duration import Duration


class WaitBehavior(py_trees.behaviour.Behaviour):
    """RUNNING until *duration_sec* has passed on the node's clock, then SUCCESS.

    The clock starts in initialise(), i.e. each time the tree reaches this
    leaf, so a re-entered sequence waits the full duration again.
    """

    def __init__(self, name: str, duration_sec: float):
        super().__init__(name)
        if (isinstance(duration_sec, bool) or not isinstance(duration_sec, (int, float))
                or not math.isfinite(duration_sec) or duration_sec < 0.0):
            raise ValueError(
                f'[{name}] duration_sec must be a finite, non-negative number of '
                f'seconds; got {duration_sec!r}')
        self.duration_sec = float(duration_sec)
        self.node = None
        self._deadline = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        return True

    def initialise(self):
        self._deadline = self.node.get_clock().now() + Duration(seconds=self.duration_sec)

    def update(self):
        if self.node.get_clock().now() < self._deadline:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self._deadline = None
