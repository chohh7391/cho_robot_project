"""Read and report the OpenArm MIT task controller's diagnostics services.

The direct MIT task controller exposes two `std_srvs/Trigger` services whose
response message is a flat key=[...] string:

    ~/task_diagnostics   last_pose_error=[6] peak_wrench=[6] peak_tau_ff=[7] q_ref=[7]
    ~/protocol_status    session= ack= safe_generation= safe_ack= status=

`peak_wrench` and `peak_tau_ff` are cumulative highs since controller
activation; they never decrease. A single reading is therefore not a
measurement of one probe. Taking a baseline before the probe and diffing
against it is what turns them into a per-probe number, which is the whole
point of this behaviour.
"""

import re

import py_trees
from std_srvs.srv import Trigger

from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior


BLACKBOARD_NAMESPACE = '/mit_tuning'

_ARRAY_RE = re.compile(r'(\w+)=\[([^\]]*)\]')
_SCALAR_RE = re.compile(r'(\w+)=([-+0-9.eE]+)(?:\s|$)')


def parse_diagnostics(message: str) -> dict:
    """Parse a `key=[a,b,c]` / `key=value` Trigger message into floats."""
    parsed = {}
    for key, body in _ARRAY_RE.findall(message or ''):
        values = []
        for item in body.split(','):
            item = item.strip()
            if not item:
                continue
            try:
                values.append(float(item))
            except ValueError:
                values = []
                break
        if values:
            parsed[key] = values
    remainder = _ARRAY_RE.sub(' ', message or '')
    for key, value in _SCALAR_RE.findall(remainder):
        try:
            parsed[key] = float(value)
        except ValueError:
            continue
    return parsed


def _fmt(values, digits=4):
    if isinstance(values, list):
        return '[' + ', '.join(f'{v:.{digits}f}' for v in values) + ']'
    return f'{values:g}'


class MitTaskDiagnosticsServiceBehavior(BaseServiceBehavior):
    """Read one MIT diagnostics service and log it, optionally against a baseline.

    `record_as` stores the parsed reading on the blackboard. `compare_to` reads
    a previously stored key and reports the per-probe growth of the cumulative
    peaks, which is the number worth comparing between gain settings.

    The behaviour never fails on content: an aborted or stalled probe is a
    legitimate tuning datapoint, and failing here would end the run before the
    return leg could bring the arm back. It fails only when the service itself
    is unreachable.
    """

    def __init__(
        self,
        name: str,
        controller_name: str,
        service: str = 'task_diagnostics',
        record_as: str = None,
        compare_to: str = None,
        timeout_sec: float = 5.0,
    ):
        super().__init__(
            name, Trigger, f'/{controller_name}/{service}', timeout_sec=timeout_sec
        )
        self.controller_name = controller_name
        self.record_as = record_as
        self.compare_to = compare_to
        self.board = py_trees.blackboard.Client(name=name, namespace=BLACKBOARD_NAMESPACE)
        for key in (record_as, compare_to):
            if key:
                self.board.register_key(key=key, access=py_trees.common.Access.WRITE)

    def handle_response(self, result):
        if not result.success:
            self.node.get_logger().warn(
                f'[{self.name}] {self.service_name} reported failure: {result.message}'
            )
            return py_trees.common.Status.SUCCESS

        reading = parse_diagnostics(result.message)
        if not reading:
            self.node.get_logger().info(f'[{self.name}] {result.message}')
            return py_trees.common.Status.SUCCESS

        lines = [f'[{self.name}] {self.service_name}']
        for key in sorted(reading):
            lines.append(f'    {key:16s} = {_fmt(reading[key])}')

        baseline = None
        if self.compare_to:
            baseline = getattr(self.board, self.compare_to, None)
        if baseline:
            lines.append('    --- growth since baseline (this probe only) ---')
            for key in ('peak_wrench', 'peak_tau_ff'):
                now, before = reading.get(key), baseline.get(key)
                if not now or not before or len(now) != len(before):
                    continue
                delta = [a - b for a, b in zip(now, before)]
                lines.append(f'    d{key:15s} = {_fmt(delta)}')
                lines.append(f'    max d{key:11s} = {max(delta):.4f}')

        self.node.get_logger().info('\n'.join(lines))

        if self.record_as:
            setattr(self.board, self.record_as, reading)
        return py_trees.common.Status.SUCCESS
