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

"""Launch-ordering helpers that are not specific to one simulator."""

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO


def start_on_output(process, marker, actions, stdout=True, stderr=False):
    """Start `actions` the first time `process` prints `marker`.

    For a process whose services come up long before it can serve them - a
    controller_manager whose hardware is still loading, or one whose realtime
    loop waits for a simulator's first /clock. A spawner started on process
    start reaches configure/switch in that window and dies, and
    --controller-manager-timeout does not help: it covers waiting for the
    services, not for what is behind them.

    ROS 2 C++ logging goes to stderr, so pass stderr=True to watch a ROS
    node's log lines (watching both keeps working if that ever changes).
    """
    started = {'done': False}

    def on_output(event):
        if started['done']:
            return None
        if marker not in event.text.decode(errors='replace'):
            return None
        started['done'] = True
        return actions

    return RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=process,
            on_stdout=on_output if stdout else None,
            on_stderr=on_output if stderr else None,
        )
    )
