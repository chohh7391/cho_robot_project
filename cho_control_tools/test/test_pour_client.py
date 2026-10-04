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

"""The pour client's command line, and the one thing about its shutdown that matters."""

import pytest

from cho_control_tools.clients.fr5 import pour_client


def test_container_defaults_to_reading_the_empty_vessel():
    args = pour_client.build_parser().parse_args(['--target', '50'])
    assert args.container == 'auto'
    assert args.material == 'liquid'
    assert args.flow_index == 0.0


def test_a_flow_index_outside_its_range_is_refused_before_anything_starts(monkeypatch):
    def must_not_run(**_kwargs):
        raise AssertionError('rclpy.init reached with an invalid flow_index')

    monkeypatch.setattr(pour_client.rclpy, 'init', must_not_run)
    assert pour_client.main(['--target', '50', '--flow-index', '3']) == 2


def test_ctrl_c_is_left_to_python_so_the_cancel_can_still_be_sent(monkeypatch):
    # With rclpy's default handler, SIGINT shuts the context down before the
    # KeyboardInterrupt reaches pour(), and the cancel it then sends goes out on
    # a dead context: the controller never hears it and pours on to target.
    seen = {}

    class Stop(Exception):
        pass

    def record(**kwargs):
        seen.update(kwargs)
        raise Stop

    monkeypatch.setattr(pour_client.rclpy, 'init', record)
    with pytest.raises(Stop):
        pour_client.main(['--target', '50', '--container', '139.15'])
    assert seen.get('signal_handler_options') == pour_client.SignalHandlerOptions.NO
