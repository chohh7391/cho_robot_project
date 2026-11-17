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

"""teleop_bridge: a teleoperation device's ROS output, streamed to the arms' action-chunk controllers.

A device driver outside this repository publishes what the operator does (for
the XR controllers, Isaac Capture's ``teleop_ros2``); the bridge maps it onto the
robot and drives each arm through the controller action contract
(cho_interfaces/CONTRACT.md): one ``ChunkStream`` goal per arm and absolute
task-space ``ActionChunk`` references, plus the gripper's ``~/width_command``.

- ``inputs/``: one module per device, decoding its topic into an ``InputFrame``.
- ``clutch``: a hand's motion since it engaged, applied to the arm's held pose.
- ``state``: start / stop from the device's buttons.
- ``chunk_streamer``: the registry's controllers for an arm, its goal and its chunks.
- ``node``: the ROS node and ``main``.

Every robot fact comes from cho_robot_config, so a project repository runs the
bridge on its own robot by pointing ``CHO_ROBOT_CONFIG_DIR`` at its registry.
Operating it: docs/teleop_bridge.md.
"""
