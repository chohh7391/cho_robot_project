import py_trees_ros
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
    
    use_sim_time = node.get_parameter("use_sim_time").get_parameter_value().bool_value
    task = node.get_parameter("task").get_parameter_value().string_value

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
        unicode_tree_debug=True
    )
    
    try:
        tree.setup(node=node, timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        node.get_logger().error(f"Setup timed out!: {e}")
        return
    
    try:
        tree.tick_tock(period_ms=100)

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin() 
        
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
