import py_trees_ros
import py_trees
import rclpy
from rclpy.executors import MultiThreadedExecutor
from cho_task_manager.tasks import build_task_tree
from cho_task_manager.utils.controller_names import load_robot_config


def main():
    rclpy.init()

    node = rclpy.create_node("task_manager_node")

    node.declare_parameter("task", "peg_insert")
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
    # task default (direct), which is right for a simulator with no
    # collisions to check.
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

    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=debug_tree
    )

    try:
        tree.setup(node=node, timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        node.get_logger().error(f"Setup timed out!: {e}")
        node.destroy_node()
        rclpy.try_shutdown()
        return
    except Exception as e:
        node.get_logger().error(f"Setup failed: {e}")
        node.destroy_node()
        rclpy.try_shutdown()
        return

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
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
        tree.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
