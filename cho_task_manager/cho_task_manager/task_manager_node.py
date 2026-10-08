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

import contextlib
import signal
import threading
import time

import py_trees_ros
import py_trees
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_task_manager.tasks import build_task_tree
from cho_task_manager.utils.controller_names import load_robot_config

# The longest shutdown keeps spinning so the cancels the action leaves still
# owe -- issued by a preemption, or armed to go out when a pending goal is
# accepted, possibly on the tree's very last tick -- actually leave the process
# and are answered before the node is destroyed. It stops as soon as none is
# outstanding. Wall time: sim time may have stopped along with the simulator.
CANCEL_FLUSH_SEC = 1.0

# The task run when none is given. run_task_manager.launch.py declares the same
# default for its `task` argument; test_task_manager_node pins the two together.
DEFAULT_TASK = "pick_place"


class StoppableBehaviourTree(py_trees_ros.trees.BehaviourTree):
    """A BehaviourTree that can be stopped from the main thread without racing a tick.

    tick_tock() ticks from an rclpy timer, which the MultiThreadedExecutor runs
    on a worker thread, while Ctrl-C lands on the main thread. Invalidating the
    root mid-tick could let that tick go on to initialise() the next leaf and
    send a goal nothing would ever cancel.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._tick_lock = threading.Lock()
        self._stopped = False

    def tick(self, *args, **kwargs):
        with self._tick_lock:
            # A tick the executor dispatched just before stop() must not undo it.
            if not self._stopped:
                super().tick(*args, **kwargs)

    def stop(self):
        """Stop ticking and invalidate a RUNNING root; True if anything was preempted.

        tree.shutdown() does not do this -- it only calls each behaviour's
        shutdown() -- so without it a RUNNING action leaf never reaches
        terminate(INVALID) and its goal keeps driving the arm after the node
        is gone.
        """
        if self.timer is not None:
            self.timer.cancel()
        with self._tick_lock:
            self._stopped = True
            if self.root.status != py_trees.common.Status.RUNNING:
                return False
            self.root.stop(py_trees.common.Status.INVALID)
            return True


def _outstanding_cancels(tree):
    """The action leaves of *tree* that still owe a cancel or await its answer."""
    return [behaviour for behaviour in tree.root.iterate()
            if isinstance(behaviour, BaseActionBehavior) and behaviour.cancels_outstanding()]


def _flush_cancels(tree, executor, node, flush_sec):
    """Spin, for at most *flush_sec* of wall time, while any leaf owes a cancel.

    Runs however the tree ended, not only after a preemption: a leaf that
    ended SUCCESS or FAILURE on the last tick with a goal still awaiting
    acceptance arms a cancel that only the executor can deliver, and
    destroying the node without spinning drops it -- the goal would then be
    accepted, and executed, after the task manager is gone.
    """
    pending = _outstanding_cancels(tree)
    if not pending:
        return
    node.get_logger().warn(
        f"Waiting up to {flush_sec:g}s for the cancels of "
        f"{', '.join(leaf.name for leaf in pending)} to be sent and answered")
    deadline = time.monotonic() + flush_sec
    while pending and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        pending = _outstanding_cancels(tree)
    if pending:
        node.get_logger().error(
            f"No answer to the cancels of {', '.join(leaf.name for leaf in pending)} "
            f"within {flush_sec:g}s: those goals may still be running")


@contextlib.contextmanager
def _signals_held():
    """Hold Ctrl-C and SIGTERM for the block; yield the list of those received.

    Only the main thread can install handlers, and a handler that was not
    installed from Python cannot be put back, so in either case nothing is
    held and the list stays empty. An interrupt that lands while the handlers
    are being installed escapes as KeyboardInterrupt before the block runs;
    shutdown_task_manager() runs the block again for that.
    """
    received = []
    signals = (signal.SIGINT, signal.SIGTERM)
    if threading.current_thread() is not threading.main_thread():
        yield received
        return
    previous = {sig: signal.getsignal(sig) for sig in signals}
    if any(handler is None for handler in previous.values()):
        yield received
        return

    def hold(signum, _frame):
        received.append(signum)

    try:
        for sig in signals:
            signal.signal(sig, hold)
        yield received
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


# How many times shutdown starts the stop-and-flush over when an interrupt
# lands before the handlers holding it are installed. Each attempt is short
# and idempotent; the bound only keeps a held-down Ctrl-C from looping forever.
_SHUTDOWN_ATTEMPTS = 5


def shutdown_task_manager(tree, executor, node, flush_sec=CANCEL_FLUSH_SEC):
    """Preempt whatever is still running, let its cancels go out, then tear down once.

    Ctrl-C and SIGTERM are held from before tree.stop() to the end of the
    flush. An interrupt during stop() used to leave every leaf after the one
    being terminated with its goal running, and one during (or held until)
    the flush skipped it -- dropping a cancel armed for a goal still awaiting
    acceptance, which the server then accepted and ran after the node was
    gone. The flush is bounded by *flush_sec*, so holding them costs at most
    that long; they take effect as the teardown.
    """
    try:
        for _attempt in range(_SHUTDOWN_ATTEMPTS):
            try:
                with _signals_held() as received:
                    preempted = tree.stop()
                    if preempted:
                        node.get_logger().warn("Stopped mid-task: cancelling the running goals")
                    _flush_cancels(tree, executor, node, flush_sec)
            except KeyboardInterrupt:
                # It landed before the handlers were in place, so the block may
                # not have run at all. Both steps are idempotent: run it again.
                continue
            if received:
                node.get_logger().warn(
                    'Interrupt held until the running goals were cancelled; shutting down')
            break
    finally:
        # Let a callback already running finish before the node goes away
        # under it, and stop the executor taking new ones.
        executor.shutdown(timeout_sec=flush_sec)
        # The node is ours, not the tree's: keep shutdown() from destroying it
        # too, so it is destroyed exactly once.
        tree.shutdown(destroy_node=False)
        node.destroy_node()
        rclpy.try_shutdown()


def _abandon_setup(tree, node, message):
    """Tear down a tree that never started ticking: nothing was sent, nothing to cancel."""
    node.get_logger().error(message)
    try:
        tree.shutdown(destroy_node=False)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # rclpy's own SIGINT/SIGTERM handler shuts the context down before the
    # KeyboardInterrupt reaches the loop below, and every cancel sent after that
    # fails on a dead context. Leave SIGINT to Python, and route SIGTERM (what
    # ros2 launch escalates to) through the same path.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _interrupt)

    node = rclpy.create_node("task_manager_node")

    # Same default as run_task_manager.launch.py's `task` argument, so starting
    # the node directly runs what the launch would.
    node.declare_parameter("task", DEFAULT_TASK)
    node.declare_parameter("robot_type", "franka")
    # Arm profile. A bimanual robot prefixes its controller names per arm, so a
    # task that hard-coded the single-arm name would look for an action server
    # that does not exist on that build.
    node.declare_parameter("arm", "single")
    # Bringup control mode. A bringup exports exactly one command interface per
    # joint, so it decides which controller can hold the arm -- and therefore
    # which one a task's safe-abort branch may switch to. Empty means "use the
    # mode the task itself is written for"; set it when the bringup was started
    # in a different one.
    node.declare_parameter("control_mode", "")
    node.declare_parameter("debug_tree", True)
    node.declare_parameter("print_tree", True)
    # Probe geometry for parameterised tuning tasks. Declared here so a gain
    # sweep never needs a source edit or a rebuild between runs; tasks that do
    # not probe simply ignore them. An all-zero translation means "use the
    # task's own default": ROS 2 cannot type an empty array parameter, and a
    # zero-length probe would be meaningless anyway.
    node.declare_parameter("probe_translation", [0.0, 0.0, 0.0])
    node.declare_parameter("probe_duration", 0.0)
    node.declare_parameter("probe_return", True)
    # Recorded-trajectory replay. Paths rather than contents: a recording is an
    # artefact produced elsewhere, and the layout file is the cell's own
    # description of itself, which the replay checks the recording against and
    # REFUSES on a mismatch. Empty means "not a replay task"; the replay tree
    # raises a clear error if it is selected without them.
    node.declare_parameter("replay_trajectory", "")
    node.declare_parameter("replay_meta", "")
    node.declare_parameter("replay_layout", "")
    # Fraction of the recorded clock to replay at. 0.0 means "use the task's own
    # default", which is deliberately conservative: a first replay should be
    # slow, and these recordings are timed for a simulator, not for this arm.
    node.declare_parameter("replay_speed_scale", 0.0)
    # How the arm reaches the recording's start pose: "direct" interpolates
    # there through the hold controller and checks nothing, "moveit" plans it
    # and needs move_group plus the MoveIt bridge running. Empty keeps the
    # task default (moveit, trajectory_replay.DEFAULT_HOME_VIA); a bringup
    # without MoveIt -- a simulator with no collisions to check -- passes
    # "direct".
    node.declare_parameter("home_via", "")
    # Vessels the perceived replay keeps a camera on while the arm runs, as a
    # space- or comma-separated list. Empty means no watchdog, which is the
    # default on purpose: a transfer recording MOVES a vessel, and a monitor
    # that did not know which one is being carried would abort the run it
    # exists to protect. Name the ones that should stay put.
    node.declare_parameter("replay_watch", "")
    # Where the wrist camera goes to look at a vessel the standing camera
    # cannot see (occlusion_recovery). A bench's joint configurations, not a
    # robot's, so it is a path like the replay artefacts above rather than
    # anything derivable from the registry. Empty means "not a recovery task";
    # the tree raises a clear error if it is selected without one.
    node.declare_parameter("sweep_config", "")
    # single_pass (empty) looks for every object in one pass over the raster;
    # per_object sweeps and returns once per object.
    node.declare_parameter("sweep_mode", "")
    # Where cho_object_pose says what each camera can see. Empty keeps the
    # node's own default, which is what its launch publishes on.
    node.declare_parameter("visibility_topic", "")

    use_sim_time = node.get_parameter("use_sim_time").get_parameter_value().bool_value
    task = node.get_parameter("task").get_parameter_value().string_value
    robot_type = node.get_parameter("robot_type").get_parameter_value().string_value
    arm = node.get_parameter("arm").get_parameter_value().string_value
    control_mode = node.get_parameter("control_mode").get_parameter_value().string_value
    debug_tree = node.get_parameter("debug_tree").get_parameter_value().bool_value
    print_tree = node.get_parameter("print_tree").get_parameter_value().bool_value

    node.get_logger().info(f"--- Running in {'SIMULATION' if use_sim_time else 'REAL'} mode ---")
    node.get_logger().info(f"--- Robot type: {robot_type} (arm profile: {arm}) ---")

    probe_translation = list(
        node.get_parameter("probe_translation").get_parameter_value().double_array_value
    )
    probe_duration = node.get_parameter("probe_duration").get_parameter_value().double_value
    probe_return = node.get_parameter("probe_return").get_parameter_value().bool_value

    try:
        robot_config = load_robot_config(robot_type, arm)
    except ValueError as e:
        node.get_logger().error(str(e))
        node.destroy_node()
        rclpy.try_shutdown()
        return

    # Only override what was actually supplied, so a task default stays in
    # force when the operator does not set the parameter.
    if any(value != 0.0 for value in probe_translation):
        robot_config['probe_translation'] = probe_translation
    if probe_duration > 0.0:
        robot_config['probe_duration'] = probe_duration
    robot_config['probe_return'] = probe_return
    if control_mode:
        robot_config['control_mode'] = control_mode
        node.get_logger().info(f"--- Control mode override: {control_mode} ---")

    for key in ("replay_trajectory", "replay_meta", "replay_layout", "home_via",
                "replay_watch", "sweep_config", "sweep_mode", "visibility_topic"):
        value = node.get_parameter(key).get_parameter_value().string_value
        if value:
            robot_config[key] = value
    replay_speed_scale = node.get_parameter(
        "replay_speed_scale").get_parameter_value().double_value
    if replay_speed_scale > 0.0:
        robot_config["replay_speed_scale"] = replay_speed_scale

    try:
        root = build_task_tree(task, robot_config)
    except ValueError as e:
        node.get_logger().error(str(e))
        node.destroy_node()
        rclpy.try_shutdown()
        return

    tree = StoppableBehaviourTree(
        root=root,
        unicode_tree_debug=debug_tree
    )

    try:
        tree.setup(node=node, timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        _abandon_setup(tree, node, f"Setup timed out!: {e}")
        return
    except KeyboardInterrupt:
        # Setup waits seconds per action server, so Ctrl-C lands here often.
        # No goal has been sent yet: a clean exit, not a traceback.
        _abandon_setup(tree, node, "Interrupted during setup; the task was not started")
        return
    except Exception as e:
        _abandon_setup(tree, node, f"Setup failed: {e}")
        return

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        terminal_status = {"status": None}

        def stop_on_terminal_status(behaviour_tree):
            root_status = behaviour_tree.root.status
            if root_status not in (
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.FAILURE,
            ):
                return
            if terminal_status["status"] is not None:
                return
            terminal_status["status"] = root_status
            if hasattr(behaviour_tree, "timer") and behaviour_tree.timer is not None:
                behaviour_tree.timer.cancel()

        tree.tick_tock(
            period_ms=100,
            post_tick_handler=stop_on_terminal_status,
        )

        while rclpy.ok() and terminal_status["status"] is None:
            executor.spin_once(timeout_sec=0.1)

        if terminal_status["status"] is not None:
            node.get_logger().info(f"Task finished with status: {terminal_status['status']}")
            if print_tree:
                tree_snapshot = py_trees.display.unicode_tree(
                    root=tree.root,
                    show_status=True,
                    visited=tree.snapshot_visitor.visited,
                    previously_visited=tree.snapshot_visitor.previously_visited,
                )
                node.get_logger().info("\n" + tree_snapshot)

    except KeyboardInterrupt:
        pass
    finally:
        shutdown_task_manager(tree, executor, node)


if __name__ == '__main__':
    main()
