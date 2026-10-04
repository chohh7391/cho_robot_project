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

"""Launch helpers shared by every cho_bringup_* package.

Only what is the same for every robot lives here. Controller name lists, the
per-arm prefixing, the MIT selection rules and anything else that is about one
robot stay in that robot's own lib/<package>/utils/launch_utils.py.
"""

from .environment import prepend_to_search_paths
from .events import start_on_output
from .isaac import (
    check_isaac_install,
    DEFAULT_ISAAC_SIM_PATH,
    gate_failure_argument,
    ISAAC_READY_MARKER,
    isaac_command_gate,
    isaac_controller_startup,
    isaac_sim_command,
    isaac_sim_process,
    shutdown_on_gate_failure,
)
from .runtime_params import (
    bringup_params,
    create_runtime_param_cleanup,
    runtime_control_mode,
    runtime_param_cleanup,
    runtime_param_dir,
    write_position_arm_param_file,
    write_runtime_param_file,
)
from .spawners import (
    chain_spawners,
    create_controller_spawners,
    make_spawner_node,
    top_level_spawner,
)
from .utils import as_bool, load_package_utils, load_yaml, strict_bool, unique_names

__all__ = [
    'as_bool',
    'bringup_params',
    'chain_spawners',
    'check_isaac_install',
    'create_controller_spawners',
    'create_runtime_param_cleanup',
    'DEFAULT_ISAAC_SIM_PATH',
    'gate_failure_argument',
    'ISAAC_READY_MARKER',
    'isaac_command_gate',
    'isaac_controller_startup',
    'isaac_sim_command',
    'isaac_sim_process',
    'load_package_utils',
    'load_yaml',
    'make_spawner_node',
    'prepend_to_search_paths',
    'runtime_control_mode',
    'runtime_param_cleanup',
    'runtime_param_dir',
    'shutdown_on_gate_failure',
    'start_on_output',
    'top_level_spawner',
    'unique_names',
    'write_position_arm_param_file',
    'write_runtime_param_file',
]
