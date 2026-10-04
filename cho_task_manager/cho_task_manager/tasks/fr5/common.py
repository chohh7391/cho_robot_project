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

"""Fragments shared by the FR5 task trees.

The same split ``tasks/franka/forge/common.py`` makes: what more than one FR5
tree needs is here, so the second tree to need it does not get to pick a
different home pose than the first one did.
"""

from collections import namedtuple

# The robot's task home, the registry's poses.task_home -- for the FR5 home 1,
# keyed the same way the action client and the MoveIt SRDF key it, never the
# diagnostic-only home 0 (fr5.yaml's schema refuses that). Re-exported here so
# the FR5 trees keep importing it from one place.
from cho_task_manager.subtrees.home import home_joint_state  # noqa: F401

# Every FR5 bringup hard-codes control_mode 'position' (cho_bringup_fr5), and
# that is also the only mode fr5.yaml declares a hold controller for.
CONTROL_MODE = 'position'


#: One tagged vessel on the bench: what it is called, where its detected pose
#: comes out, and the blackboard entry a tree latches it into.
VesselSpec = namedtuple('VesselSpec', 'name topic key')

#: The vessels the cameras track.
#:
#: ``name`` is the joining key and has to be the same word in three places: the
#: object table (``config/perception/vessel_detect.yaml``), which decides the
#: topic; the layout a recording assumes
#: (``layout_the_trajectory_assumes`` in its meta) and the cell layout files in
#: ``config/replay/``, which is what makes a detected pose checkable against a
#: replay. Tests assert the first of those; the recordings are produced
#: elsewhere, so the third is checked at run time by the layout gate saying an
#: object is unverified rather than by anything failing here.
#:
#: Shared rather than repeated because two trees use it -- vessel_detect
#: latches them, perceived_replay checks a recording against them -- and a
#: second copy would be a topic string that can go stale in one of the two.
VESSELS = (
    VesselSpec('beaker', '/perception/object_pose/beaker', 'beaker_pose'),
    VesselSpec('flask', '/perception/object_pose/flask', 'flask_pose'),
)
