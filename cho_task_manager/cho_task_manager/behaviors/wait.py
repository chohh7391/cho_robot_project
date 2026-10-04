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

from cho_task_manager.utils.clock import arm, deadline_after


class WaitBehavior(py_trees.behaviour.Behaviour):
    """RUNNING until *duration_sec* has passed on the node's clock, then SUCCESS.

    The clock starts in initialise(), i.e. each time the tree reaches this
    leaf, so a re-entered sequence waits the full duration again.

    A node clock reading zero is sim time before the first /clock message.
    A deadline taken from it would be met the moment /clock arrives -- the
    simulator's time is already far past zero -- so the wait would end without
    any physics having run. Until the clock reads something else the leaf
    stays RUNNING and sets no deadline; the wait starts from the first real
    reading.
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
        # None while the clock reads 0 (utils/clock.py); update() takes it then.
        self._deadline = deadline_after(self.node.get_clock(), self.duration_sec)

    def update(self):
        clock = self.node.get_clock()
        self._deadline = arm(self._deadline, clock, self.duration_sec)
        if self._deadline is None:
            self.feedback_message = 'waiting for the clock (no /clock yet)'
            return py_trees.common.Status.RUNNING
        if clock.now() < self._deadline:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self._deadline = None
