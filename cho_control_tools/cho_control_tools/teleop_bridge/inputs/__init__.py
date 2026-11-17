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

"""The devices the bridge reads, by the name ``--input`` takes.

Each input module has ``SIDES`` (the hands it reports), ``BUTTONS`` (the names
its frames carry, as ``<side>_<button>``) and ``decode(data, pose_source,
button_threshold)``, which turns its message's payload into an ``InputFrame``.
"""

INPUTS = ('xr_controller',)


def load_input(name):
    """The input module named *name*."""
    if name == 'xr_controller':
        from cho_control_tools.teleop_bridge.inputs import xr_controller
        return xr_controller
    raise ValueError(f"unknown input '{name}'; expected one of {list(INPUTS)}")
