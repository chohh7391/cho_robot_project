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

"""The per-launch controller parameter file every bringup generates.

bringup_type, control_mode and ee_name are the same for every controller of a
launch but differ between launches, and the motion limits come from the
robot's MoveIt package rather than from a controllers.yaml. So each launch
writes them to one parameter file, hands that file to the controller_manager
(or, for an in-simulator manager, to the spawners with -p), and deletes it on
shutdown.
"""

from copy import deepcopy
import os
import tempfile

from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown

import yaml


def runtime_control_mode(control_mode):
    """The controllers' spelling of a launch's control_mode: torque is 'effort'."""
    return 'effort' if control_mode == 'torque' else control_mode


def bringup_params(bringup_type, control_mode, ee_name=None):
    """The parameters a launch injects into its controllers.

    control_mode is the launch's spelling (torque | position | velocity) and is
    translated with runtime_control_mode(). ee_name is left out when empty: a
    bimanual build sets one per arm in its controllers file, and overriding it
    from here would point both arms at one hand.
    """
    params = {
        'bringup_type': bringup_type,
        'control_mode': runtime_control_mode(control_mode),
    }
    if ee_name:
        params['ee_name'] = ee_name
    return params


def runtime_param_dir():
    """ROS_HOME, or ~/.ros when it is unset.

    Not /tmp: the file is read by a controller_manager this same launch starts,
    and /tmp is not reliably shared between processes under containers or a
    systemd PrivateTmp unit.
    """
    return os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')


def write_runtime_param_file(controller_params, shared_params=None, base=None,
                             prefix='cho_runtime_params_'):
    """Write `{'/**': {controller: {'ros__parameters': ...}}}` and return its path.

    controller_params maps each controller name to its own parameters. Every
    controller additionally gets a deep copy of `shared_params` (the robot's
    motion limits); a controller's own parameters win on a shared key.

    A copy each, because yaml writes a dict that two controllers share as a
    YAML anchor and alias, and rcl's parameter parser rejects aliases - the
    controller_manager then dies at startup.

    `base` is an already-loaded parameter document the controller entries are
    merged into (cho_bringup_franka's payload.yaml); it is not modified.
    """
    document = deepcopy(base) if base else {}
    wildcard = document.setdefault('/**', {})
    for name, params in controller_params.items():
        ros_params = wildcard.setdefault(name, {}).setdefault('ros__parameters', {})
        if shared_params:
            ros_params.update(deepcopy(shared_params))
        ros_params.update(deepcopy(params))

    directory = runtime_param_dir()
    os.makedirs(directory, exist_ok=True)
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix=prefix, dir=directory)
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump(document, stream)
    return path


def create_runtime_param_cleanup(runtime_param_file):
    """An OpaqueFunction that deletes `runtime_param_file` if it still exists."""
    def cleanup(context, *args, **kwargs):
        del context, args, kwargs
        if os.path.exists(runtime_param_file):
            os.unlink(runtime_param_file)
        return []

    return OpaqueFunction(function=cleanup)


def runtime_param_cleanup(runtime_param_file):
    """Delete `runtime_param_file` when the launch shuts down."""
    return RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[create_runtime_param_cleanup(runtime_param_file)],
        )
    )
