"""
Send one pour and report what it actually did.

This is the commissioning tool for PouringController, and the numbers it prints
are the ones the material profiles in ``controllers.yaml`` are set from. Only
``liquid.free`` in that file has hardware behind it; every other endpoint is a
seed, and the way to replace a seed is to pour with it and read
``measured_afterflow`` off the result.

    # water into a flask that weighs 139.15 g
    ros2 run cho_control_tools fr5_pour_client --target 50 --container 139.15

    # let it read the empty vessel itself
    ros2 run cho_control_tools fr5_pour_client --target 50 --container auto

    # thick syrup: same class, further along it
    ros2 run cho_control_tools fr5_pour_client --target 50 --container auto \
        --material liquid --flow-index 0.8

    # dry sugar
    ros2 run cho_control_tools fr5_pour_client --target 30 --container auto \
        --material granular --flow-index 0.1

Ctrl-C cancels. It does not abandon the vessel mid-tip: the cancel goes to the
controller, which parks it at the attitude it was carried in before the goal
ends.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from cho_interfaces.action import Pour
from cho_interfaces.msg import ScaleReading

ACTION_NAME = '/controller_action_server/pouring_controller'
SCALE_TOPIC = '/scale/reading'

MATERIALS = {
    'liquid': Pour.Goal.MATERIAL_LIQUID,
    'granular': Pour.Goal.MATERIAL_GRANULAR,
}

PHASES = {
    Pour.Feedback.PHASE_VERIFY: 'verify',
    Pour.Feedback.PHASE_SEEK: 'seek',
    Pour.Feedback.PHASE_BULK: 'bulk',
    Pour.Feedback.PHASE_RETRACT: 'retract',
    Pour.Feedback.PHASE_SETTLE: 'settle',
    Pour.Feedback.PHASE_TRIM: 'trim',
    Pour.Feedback.PHASE_DONE: 'done',
}


class PourClient(Node):

    def __init__(self):
        super().__init__('fr5_pour_client')
        self._client = ActionClient(self, Pour, ACTION_NAME)
        self._reading: ScaleReading | None = None
        self.create_subscription(
            ScaleReading, SCALE_TOPIC, self._on_scale, qos_profile_sensor_data)
        self._last_phase = None

    def _on_scale(self, msg: ScaleReading) -> None:
        self._reading = msg

    def read_empty_vessel(self, settle_sec: float = 2.0, timeout_sec: float = 15.0):
        """
        Wait for a settled reading and return it, or None.

        Deliberately strict about *settled*: this number becomes the pour's only
        zero, and taking it while the pan is still moving bakes that error into
        every gram the pour reports. It is the same discipline the controller
        applies before its first tilt.
        """
        deadline = time.time() + timeout_sec
        stable_since = None
        last = None
        print(f'Reading the empty vessel on {SCALE_TOPIC} ...')
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._reading is None:
                continue
            grams = self._reading.grams
            if last is not None and abs(grams - last) < 1e-9 and self._reading.stable:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since >= settle_sec:
                    return grams
            else:
                stable_since = None
            last = grams
        if self._reading is None:
            print(f'No reading on {SCALE_TOPIC}. Is the driver up, and the relay '
                  '(`ros2 run cho_control_tools scale_relay`) running?', file=sys.stderr)
        else:
            print('The scale never settled; something on the pan is moving.', file=sys.stderr)
        return None

    def pour(self, goal: Pour.Goal) -> int:
        if not self._client.wait_for_server(timeout_sec=5.0):
            print(f'No action server at {ACTION_NAME}. Is pouring_controller active?',
                  file=sys.stderr)
            return 1

        print(f'\nPouring {goal.target_grams:.2f} g of '
              f'{"granular media" if goal.material else "liquid"} '
              f'(flow_index {goal.flow_index:.2f}) into a {goal.container_grams:.2f} g vessel\n')

        send = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('Goal rejected. The reason is on the controller\'s log.', file=sys.stderr)
            return 1

        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            print('\nCancelling; the controller parks the vessel before it ends.')
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel)
            rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        print()
        print(f'  delivered        {result.final_grams:8.2f} g   '
              f'(target {goal.target_grams:.2f}, off by '
              f'{result.final_grams - goal.target_grams:+.2f})')
        print(f'  peak tilt        {result.peak_tilt * 57.2957795:8.2f} deg')
        print(f'  elapsed          {result.elapsed:8.1f} s')
        print(f'  trim pulses      {result.trim_pulses:8d}')
        print(f'  measured tail    {result.measured_afterflow:8.2f} g   '
              '<- the number to put in this material\'s afterflow_grams')
        if result.is_completed:
            print('\n  SUCCESS')
            return 0
        print(f'\n  FAILED: {result.message}')
        return 1

    def _on_feedback(self, feedback) -> None:
        fb = feedback.feedback
        phase = PHASES.get(fb.phase, '?')
        if phase != self._last_phase:
            print(f'\n[{phase}]', end='', flush=True)
            self._last_phase = phase
        print(f'\r[{phase:8s}] {fb.current_grams:7.2f} g  '
              f'{fb.flow_rate:6.2f} g/s  tilt {fb.tilt * 57.2957795:6.2f} deg  '
              f'{fb.elapsed:5.1f} s   ', end='', flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Pour a measured amount and report what it actually did.')
    parser.add_argument('--target', type=float, required=True,
                        help='grams to deliver')
    parser.add_argument('--container', default='auto',
                        help='weight of the EMPTY receiving vessel in grams, or "auto" to read '
                             'it off the scale now. It is the pour\'s only zero -- the scale '
                             'cannot be tared over RS232 -- and the controller refuses the goal '
                             'if what is on the pan does not match it.')
    parser.add_argument('--material', choices=sorted(MATERIALS), default='liquid')
    parser.add_argument('--flow-index', type=float, default=0.0,
                        help='0..1 within the material class: 0 is water or dry salt, 1 is '
                             'honey or a damp clumping powder (default: 0)')
    parser.add_argument('--tolerance', type=float, default=0.0,
                        help='grams; 0 uses the controller\'s. It is floored by what one dose '
                             'of this material weighs.')
    parser.add_argument('--max-tilt', type=float, default=0.0,
                        help='rad from the carried attitude; 0 uses the controller\'s')
    parser.add_argument('--max-tilt-rate', type=float, default=0.0,
                        help='rad/s; 0 uses the material profile\'s')
    parser.add_argument('--timeout', type=float, default=0.0,
                        help='seconds; 0 uses the controller\'s')
    parser.add_argument('--reference-joints', type=float, nargs=6, default=None,
                        metavar='Q',
                        help='how to tip, shown rather than named: the arm at the deepest tilt '
                             'of a recorded pour (j1..j6, rad). The controller tips about the '
                             'axis the EE turns about to get there. Omitted: about the pour '
                             'joint\'s own axis.')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if not 0.0 <= args.flow_index <= 1.0:
        print('--flow-index is a 0..1 position between this material class\'s two configured '
              'endpoints, not a physical unit.', file=sys.stderr)
        return 2

    # rclpy's own SIGINT handler shuts the context down BEFORE KeyboardInterrupt
    # reaches the except in pour(), so the cancel it sends would go out on a
    # dead context and never reach the controller -- which would then pour on
    # to target with the operator believing it had stopped. Leaving SIGINT to
    # Python keeps the context alive long enough to deliver the cancel.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PourClient()
    try:
        if str(args.container).lower() == 'auto':
            container = node.read_empty_vessel()
            if container is None:
                return 1
            print(f'Empty vessel reads {container:.2f} g\n')
        else:
            container = float(args.container)

        goal = Pour.Goal()
        goal.target_grams = float(args.target)
        goal.container_grams = float(container)
        goal.material = MATERIALS[args.material]
        goal.flow_index = float(args.flow_index)
        goal.tolerance = float(args.tolerance)
        goal.max_tilt = float(args.max_tilt)
        goal.max_tilt_rate = float(args.max_tilt_rate)
        goal.timeout = float(args.timeout)
        if args.reference_joints is not None:
            goal.pour_reference_joints = [float(q) for q in args.reference_joints]
        return node.pour(goal)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
