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

"""Read and report the OpenArm MIT task controller's diagnostics services.

The direct MIT task controller exposes two `std_srvs/Trigger` services whose
success message is a flat, space-separated list of `key=value` fields
(:data:`FORMATS`, written down from cho_controller_openarm_mit):

    ~/task_diagnostics   last_pose_error=[6] peak_wrench=[6] peak_tau_ff=[7] q_ref=[7]
    ~/protocol_status    session= ack= safe_generation= safe_ack= status= controller_active=

`peak_wrench` and `peak_tau_ff` are cumulative highs since controller
activation; they never decrease. A single reading is therefore not a
measurement of one probe. Taking a baseline before the probe and diffing
against it is what turns them into a per-probe number, which is the whole
point of this behaviour.

The messages are parsed strictly, in one place (:func:`parse_diagnostics`): a
message that is not exactly the format written down here -- a field renamed,
added or reordered, an array of another length, a value that is not a number --
raises instead of being read as far as it happens to match. A controller and a
task manager from different versions would otherwise compare a baseline against
the wrong field, or against nothing, with no sign of it.
"""

import py_trees
from std_srvs.srv import Trigger

from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior
from cho_task_manager.utils.blackboard import read_if_set


BLACKBOARD_NAMESPACE = '/mit_tuning'

#: The success message of each service, field by field in the order the
#: controller writes them: (name, element count), the count None for a scalar.
#: From cho_controller_openarm_mit's task_space_impedance_controller.cpp
#: (~/task_diagnostics) and direct_controller.cpp (~/protocol_status);
#: test_mit_task_tuning checks this table against those sources. Every value is
#: a C++ double through std::ostream, so 'nan', '-nan' and 'inf' can appear and
#: are numbers here too.
FORMATS = {
    'task_diagnostics': (
        ('last_pose_error', 6), ('peak_wrench', 6), ('peak_tau_ff', 7), ('q_ref', 7)),
    'protocol_status': (
        ('session', None), ('ack', None), ('safe_generation', None), ('safe_ack', None),
        ('status', None), ('controller_active', None)),
}


class DiagnosticsFormatError(ValueError):
    """A diagnostics message that is not the format :data:`FORMATS` describes."""


def _number(service, name, text, message):
    try:
        return float(text)
    except ValueError:
        raise DiagnosticsFormatError(
            f'{service}: {name} carries {text!r}, not a number, in {message!r}') from None


def parse_diagnostics(service: str, message: str) -> dict:
    """The fields of *service*'s success *message*, as floats (lists for the arrays).

    Raises DiagnosticsFormatError for a service with no known format and for any
    message that is not exactly that format.
    """
    if service not in FORMATS:
        raise DiagnosticsFormatError(
            f'no known message format for {service!r}; known: {sorted(FORMATS)}')
    expected = FORMATS[service]
    fields = (message or '').split()
    if [field.partition('=')[0] for field in fields] != [name for name, _ in expected]:
        raise DiagnosticsFormatError(
            f'{service} answered {message!r}; expected the fields '
            f"{' '.join(name + '=' for name, _ in expected)} in that order")
    parsed = {}
    for field, (name, count) in zip(fields, expected):
        value = field.partition('=')[2]
        if count is None:
            parsed[name] = _number(service, name, value, message)
            continue
        if not (value.startswith('[') and value.endswith(']')):
            raise DiagnosticsFormatError(f'{service}: {name} is not a [..] array in {message!r}')
        items = value[1:-1].split(',') if value != '[]' else []
        if len(items) != count:
            raise DiagnosticsFormatError(
                f'{service}: {name} has {len(items)} values, expected {count}, in {message!r}')
        parsed[name] = [_number(service, name, item, message) for item in items]
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

    The behaviour never fails on a reading: an aborted or stalled probe is a
    legitimate tuning datapoint, and failing here would end the run before the
    return leg could bring the arm back. It fails when the service is
    unreachable, and when it answers in a format this does not know -- which
    the first read, before the probe moves anything, already shows.
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
        if service not in FORMATS:
            raise ValueError(
                f'[{name}] no known message format for {service!r}; known: {sorted(FORMATS)}')
        super().__init__(
            name, Trigger, f'/{controller_name}/{service}', timeout_sec=timeout_sec
        )
        self.service = service
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

        try:
            reading = parse_diagnostics(self.service, result.message)
        except DiagnosticsFormatError as error:
            self.node.get_logger().error(
                f'[{self.name}] cannot read {self.service_name}: {error}. Is the controller '
                'from the same version as this task manager?')
            return py_trees.common.Status.FAILURE

        lines = [f'[{self.name}] {self.service_name}']
        for key in sorted(reading):
            lines.append(f'    {key:16s} = {_fmt(reading[key])}')

        # read_if_set, not getattr(..., None): a baseline whose reading never got
        # recorded is a registered-but-unwritten key, and the KeyError py_trees
        # raises for it is not absorbed by getattr's default -- it would escape
        # the tick and take the node down instead of just skipping the diff.
        baseline = read_if_set(self.board, self.compare_to)
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
