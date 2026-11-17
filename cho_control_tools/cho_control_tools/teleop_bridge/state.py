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

"""Start and stop from the device's buttons.

Two states. ``stopped``: no arm has a goal. ``running``: every arm has a
``ChunkStream`` goal and holds its target, moving only while its clutch is
engaged. The run-toggle button's press (rising edge) switches between them;
the kill button stops whenever it is down, and wins over the toggle.

A button already down when the machine starts, or when input comes back after
a gap, counts only once it has been released: a held button never reads as a
fresh press.
"""

from __future__ import annotations

STOPPED = 'stopped'
RUNNING = 'running'
START = 'start'
STOP = 'stop'


class RunState:
    """The bridge's run state, driven by two named buttons of an InputFrame."""

    def __init__(self, run_toggle: str, kill: str):
        if not run_toggle or not kill or run_toggle == kill:
            raise ValueError('run_toggle and kill must be two different buttons')
        self._run_toggle = run_toggle
        self._kill = kill
        self.state = STOPPED
        self._toggle_was_down = True

    @property
    def running(self) -> bool:
        return self.state == RUNNING

    def update(self, buttons):
        """Advance with one frame's *buttons* (None while there is no input); return START, STOP or None."""
        if buttons is None:
            self._toggle_was_down = True
            return None
        toggle = bool(buttons.get(self._run_toggle, False))
        kill = bool(buttons.get(self._kill, False))
        pressed = toggle and not self._toggle_was_down
        self._toggle_was_down = toggle
        if kill:
            return self.stop()
        if pressed:
            if self.state == STOPPED:
                self.state = RUNNING
                return START
            return self.stop()
        return None

    def stop(self):
        """Go to stopped; STOP if that changed the state, else None."""
        if self.state == STOPPED:
            return None
        self.state = STOPPED
        return STOP
