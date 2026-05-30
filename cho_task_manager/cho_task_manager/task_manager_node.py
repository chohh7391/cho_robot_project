import py_trees_ros
import py_trees
import rclpy
from rclpy.executors import MultiThreadedExecutor
from cho_task_manager.tasks import (
    create_pick_and_place_tree,
    create_forge_tree,
)


def main():
    rclpy.init()

    node = rclpy.create_node("task_manager_node")

    node.declare_parameter("task", "pick_and_place")
    node.declare_parameter("debug_tree", True)
    node.declare_parameter("print_tree", True)
    
    use_sim_time = node.get_parameter("use_sim_time").get_parameter_value().bool_value
    task = node.get_parameter("task").get_parameter_value().string_value
    debug_tree = node.get_parameter("debug_tree").get_parameter_value().bool_value
    print_tree = node.get_parameter("print_tree").get_parameter_value().bool_value

    node.get_logger().info(f"--- Running in {'SIMULATION' if use_sim_time else 'REAL'} mode ---")
    
    if task.lower() == "pick_and_place":
        root = create_pick_and_place_tree()
    elif task.lower() == "forge":
        root = create_forge_tree()
    else:
        raise ValueError(f"Invalid task: {task}")
    
    # Create BehaviourTree
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
