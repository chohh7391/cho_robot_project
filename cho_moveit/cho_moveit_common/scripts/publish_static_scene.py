#!/usr/bin/env python3
"""Apply the static floor, optionally unlock trajectory execution, and report readiness."""

import math
import sys
import time

from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger


class StaticSceneGate(Node):
    def __init__(self):
        super().__init__('fr5_static_planning_scene')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('object_id', 'floor')
        self.declare_parameter('size_csv', '4.0,4.0,0.10')
        self.declare_parameter('position_csv', '0.0,0.0,-0.05')
        self.declare_parameter('max_attempts', 10)
        self.declare_parameter('service_timeout', 2.0)
        self.declare_parameter('retry_delay', 0.5)
        self.declare_parameter('activate_controller', '')
        self.declare_parameter('deactivate_controller', '')
        activate = self.declare_parameter(
            'activate_controllers', Parameter.Type.STRING_ARRAY)
        deactivate = self.declare_parameter(
            'deactivate_controllers', Parameter.Type.STRING_ARRAY)
        # A typed declaration without an override remains NOT_SET in Humble;
        # get_parameter(...).value then raises ParameterUninitializedException.
        # Explicitly initialize the optional arrays while retaining their static
        # STRING_ARRAY descriptors. This also avoids launch_ros's empty-list ->
        # tuple evaluation bug at the launch boundary.
        defaults = []
        if activate.type_ == Parameter.Type.NOT_SET:
            defaults.append(Parameter(
                'activate_controllers', Parameter.Type.STRING_ARRAY, []))
        if deactivate.type_ == Parameter.Type.NOT_SET:
            defaults.append(Parameter(
                'deactivate_controllers', Parameter.Type.STRING_ARRAY, []))
        if defaults:
            self.set_parameters(defaults)
        self.declare_parameter('controller_ready_timeout', 90.0)
        self.declare_parameter('switch_response_timeout', 15.0)
        self.declare_parameter('ready_service', '/static_scene_ready')
        self._ready = False
        self._ready_service = self.create_service(
            Trigger, self.get_parameter('ready_service').value, self._handle_ready
        )

    @staticmethod
    def _vector(value, name):
        try:
            result = [float(item.strip()) for item in value.split(',')]
        except ValueError as error:
            raise ValueError(f'{name} must be three comma-separated numbers') from error
        if len(result) != 3 or not all(math.isfinite(item) for item in result):
            raise ValueError(f'{name} must be three finite comma-separated numbers')
        return result

    def _handle_ready(self, _request, response):
        response.success = self._ready
        response.message = 'Static planning scene is ready' if self._ready else 'not ready'
        return response

    def _call_with_retry(self, client, request, operation, response_ok=lambda _result: True):
        attempts = int(self.get_parameter('max_attempts').value)
        timeout = float(self.get_parameter('service_timeout').value)
        delay = float(self.get_parameter('retry_delay').value)
        for attempt in range(1, attempts + 1):
            if not rclpy.ok():
                return None
            if client.wait_for_service(timeout_sec=timeout):
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
                if future.done() and future.exception() is None:
                    result = future.result()
                    if response_ok(result):
                        return result
                    detail = 'service rejected request'
                else:
                    detail = str(future.exception()) if future.done() else 'response timeout'
            else:
                detail = 'service unavailable'
            self.get_logger().warn(
                f'{operation} attempt {attempt}/{attempts} failed: {detail}'
            )
            if attempt < attempts:
                time.sleep(delay)
        return None

    def _controller_states(self, client, timeout):
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception() is not None:
            future.cancel()
            return None
        return {controller.name: controller.state for controller in future.result().controller}

    def _wait_for_controllers(self, client, activate, deactivate):
        deadline = time.monotonic() + float(
            self.get_parameter('controller_ready_timeout').value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            states = self._controller_states(client, 2.0)
            required = list(activate) + list(deactivate)
            if states is not None and all(name in states for name in required):
                return states
            time.sleep(0.5)
        return None

    def _switch_controllers(self, activate, deactivate):
        list_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        states = self._wait_for_controllers(list_client, activate, deactivate)
        if states is None:
            raise RuntimeError(
                'controller readiness timeout: required controllers were not loaded'
            )
        if (all(states.get(name) == 'active' for name in activate)
                and all(states.get(name) != 'active' for name in deactivate)):
            return

        switch_request = SwitchController.Request()
        switch_request.activate_controllers = list(activate)
        switch_request.deactivate_controllers = list(deactivate)
        switch_request.strictness = SwitchController.Request.STRICT
        switch_request.activate_asap = True
        switch_request.timeout.sec = 10
        switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller'
        )
        response_timeout = float(self.get_parameter('switch_response_timeout').value)
        if response_timeout <= 10.0:
            raise ValueError('switch_response_timeout must exceed the 10 second switch timeout')
        if not switch_client.wait_for_service(timeout_sec=response_timeout):
            raise RuntimeError('controller switch service unavailable')
        future = switch_client.call_async(switch_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=response_timeout)
        if future.done() and future.exception() is None and future.result().ok:
            return

        # Never submit a second overlapping switch. Cancel the local future and
        # poll actual controller state: the controller_manager may have completed
        # the first request just as the client response timed out.
        if not future.done():
            future.cancel()
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            states = self._controller_states(list_client, 2.0)
            if (states is not None
                    and all(states.get(name) == 'active' for name in activate)
                    and all(states.get(name) != 'active' for name in deactivate)):
                return
            time.sleep(0.5)
        raise RuntimeError('trajectory controller switch failed or timed out')

    def initialize(self):
        size = self._vector(self.get_parameter('size_csv').value, 'floor_size')
        position = self._vector(self.get_parameter('position_csv').value, 'floor_position')
        if any(value <= 0.0 for value in size):
            raise ValueError('floor_size values must be positive')

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.get_parameter('frame_id').value
        collision_object.id = self.get_parameter('object_id').value
        collision_object.operation = CollisionObject.ADD
        primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene(is_diff=True)
        request.scene.world.collision_objects.append(collision_object)
        apply_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        result = self._call_with_retry(
            apply_client, request, 'apply planning scene', lambda response: response.success
        )
        if result is None:
            raise RuntimeError('MoveIt did not accept the static floor planning scene')

        activate = list(self.get_parameter('activate_controllers').value)
        deactivate = list(self.get_parameter('deactivate_controllers').value)
        if not activate and self.get_parameter('activate_controller').value:
            activate = [self.get_parameter('activate_controller').value]
        if not deactivate and self.get_parameter('deactivate_controller').value:
            deactivate = [self.get_parameter('deactivate_controller').value]
        if activate:
            self._switch_controllers(activate, deactivate)

        self._ready = True
        self.get_logger().info(
            'READY: static floor is active and the MoveIt execution gate is open'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StaticSceneGate()
    exit_code = 0
    try:
        node.initialize()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:  # noqa: BLE001 - launch must see any gate failure
        node.get_logger().fatal(f'Static planning scene gate failed: {error}')
        exit_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
