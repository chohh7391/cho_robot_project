import py_trees_ros
import rclpy
from rclpy.executors import MultiThreadedExecutor
from cho_task_manager.tasks.pick_and_place import create_pick_and_place_tree

def main():
    rclpy.init()

    node = rclpy.create_node("task_manager_node")
    
    use_sim_time = node.get_parameter("use_sim_time").get_parameter_value().bool_value
    node.get_logger().info(f"--- Running in {'SIMULATION' if use_sim_time else 'REAL'} mode ---")
    
    root = create_pick_and_place_tree()
    # create your task tree in tasks folder like pick_and_place.py
    # root = create_??_tree()

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