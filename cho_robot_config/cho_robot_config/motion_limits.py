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

"""The robot's MoveIt limits, as parameters for its ros2_control controllers.

MoveIt's joint_limits.yaml is ros2_control's joint_limits parameter schema, so
the same file bounds the planner and the controllers' point-to-point
trajectories (cho_controller_common/trajectory/motion_limits_params.hpp reads
both namespaces). Pilz's pilz_cartesian_limits.yaml, when the MoveIt package
has one, bounds the Cartesian goals the same way. The bringups merge these into
each controller's runtime parameters, so no limit is copied into a
controllers.yaml.
"""

from pathlib import Path

import yaml

from .registry import load_robot_config


def _as_floats(tree):
    # A controller declares these as doubles; a YAML integer (`max_jerk: 5000`)
    # would come in as an integer parameter and fail that declaration.
    if isinstance(tree, dict):
        return {key: _as_floats(value) for key, value in tree.items()}
    if isinstance(tree, int) and not isinstance(tree, bool):
        return float(tree)
    return tree


def motion_limit_parameters(robot_type, joint_limits_file='joint_limits.yaml', moveit_config_dir=None):
    """Return {'joint_limits': ..., 'cartesian_limits': ...} for *robot_type*.

    Read from its MoveIt package's config directory (``moveit_config_dir``
    overrides it). ``cartesian_limits`` is left out when that package has no
    pilz_cartesian_limits.yaml.

    Merge a deep copy into each controller's parameters: yaml.dump writes a dict
    shared by two controllers as a YAML alias, which rcl's params parser rejects
    (the controller_manager then dies at startup).
    """
    if moveit_config_dir is None:
        from ament_index_python.packages import get_package_share_directory
        package = load_robot_config(robot_type)['moveit']['config_package']
        moveit_config_dir = Path(get_package_share_directory(package)) / 'config'
    moveit_config_dir = Path(moveit_config_dir)

    with (moveit_config_dir / joint_limits_file).open(encoding='utf-8') as stream:
        joint_limits = (yaml.safe_load(stream) or {}).get('joint_limits')
    if not isinstance(joint_limits, dict) or not joint_limits:
        raise ValueError(f'{moveit_config_dir / joint_limits_file}: no joint_limits mapping')
    parameters = {'joint_limits': _as_floats(joint_limits)}

    cartesian_path = moveit_config_dir / 'pilz_cartesian_limits.yaml'
    if cartesian_path.is_file():
        with cartesian_path.open(encoding='utf-8') as stream:
            cartesian = (yaml.safe_load(stream) or {}).get('cartesian_limits')
        if isinstance(cartesian, dict) and cartesian:
            parameters['cartesian_limits'] = _as_floats(cartesian)
    return parameters
