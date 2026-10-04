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

"""The FORGE assembly tasks: peg insertion, gear meshing, nut threading.

This file exists so the directory is a package. Without it `find_packages()`
skips `forge` entirely, so nothing here is installed and importing the franka
tasks -- which every task import goes through -- dies with
`ModuleNotFoundError: No module named 'cho_task_manager.tasks.franka.forge'`.
That stayed hidden for as long as a stale install tree kept serving the files,
and surfaced the first time the package was rebuilt from clean.
"""

from .gear_mesh import create_franka_gear_mesh_tree
from .nut_thread import create_franka_nut_thread_tree
from .peg_insert import create_franka_peg_insert_tree

__all__ = [
    'create_franka_gear_mesh_tree',
    'create_franka_nut_thread_tree',
    'create_franka_peg_insert_tree',
]
