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

"""The bridge's settings: the input's default file, a project's file merged over it.

``config/teleop_bridge/<input>.yaml`` in this package holds every key with its
default and the meaning of each; a project's ``teleop_config`` file names only
what it changes, merged over the defaults key by key (a mapping merges, any
other value replaces). Unknown keys are refused, so a misspelt key cannot
silently leave a default in force.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from cho_control_tools.teleop_bridge.clutch import ClutchConfig


@dataclass(frozen=True)
class ArmSettings:
    hand: str
    clutch: ClutchConfig


@dataclass(frozen=True)
class BridgeConfig:
    rate_hz: float
    input_topic: str
    input_timeout_sec: float
    pose_source: str
    button_threshold: float
    run_toggle: str
    kill: str
    stop_after_input_lost_sec: float
    stream_timeout_sec: float
    horizon_steps: int
    gripper_enabled: bool
    arms: dict

    @property
    def control_dt(self) -> float:
        return 1.0 / self.rate_hz

    def arm(self, profile):
        """The settings of the arm the registry calls *profile*."""
        if profile not in self.arms:
            raise ValueError(
                f"teleop_bridge has no settings for the arm '{profile}': add arms.{profile}.hand "
                f'to the teleop_config file (known: {sorted(self.arms)})')
        return self.arms[profile]


def default_config_path(input_name: str) -> Path:
    """This package's defaults for *input_name*: the installed share, else the source tree."""
    relative = Path('config') / 'teleop_bridge' / f'{input_name}.yaml'
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('cho_control_tools')) / relative
        if installed.is_file():
            return installed
    except (ImportError, LookupError):
        pass
    return Path(__file__).resolve().parents[2] / relative


def merge(base, override, path='teleop_bridge'):
    """*override* merged over *base*; a key *base* lacks is refused."""
    if not isinstance(override, dict):
        raise ValueError(f'{path} must be a mapping')
    merged = dict(base)
    for key, value in override.items():
        if key not in base:
            raise ValueError(f"unknown key {path}.{key} (known: {sorted(base)})")
        if isinstance(base[key], dict) and key != 'arms':
            merged[key] = merge(base[key], value, f'{path}.{key}')
        elif key == 'arms':
            if not isinstance(value, dict):
                raise ValueError(f'{path}.arms must be a mapping')
            merged[key] = {**base[key], **value}
        else:
            merged[key] = value
    return merged


def read_document(path) -> dict:
    with open(path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict) or set(document) != {'teleop_bridge'}:
        raise ValueError(f'{path}: expected one top-level key, teleop_bridge')
    return document['teleop_bridge']


def load_config(input_module, input_name, path=None) -> BridgeConfig:
    """The defaults for *input_name*, with the file at *path* (if any) merged over them."""
    settings = read_document(default_config_path(input_name))
    if path:
        settings = merge(settings, read_document(path))
    return parse(settings, input_module)


def parse(settings, input_module) -> BridgeConfig:
    """A BridgeConfig from the merged mapping; *input_module* names the valid hands and buttons."""
    stream = settings['stream']
    source = settings['input']
    buttons = settings['buttons']
    names = set(input_module.button_names())
    for role in ('run_toggle', 'kill'):
        if buttons[role] not in names:
            raise ValueError(f"buttons.{role} '{buttons[role]}' is not a button of this input: {sorted(names)}")
    horizon = stream['horizon_steps']
    if isinstance(horizon, bool) or not isinstance(horizon, int) or horizon < 1:
        raise ValueError('stream.horizon_steps must be an integer >= 1')
    mapping = settings['mapping']
    clutch = settings['clutch']
    rotation = (Rotation.from_euler('z', _number(mapping, 'operator_yaw_deg'), degrees=True).as_matrix()
                @ np.asarray(mapping['xr_to_base'], dtype=float))
    arms = {}
    for profile, arm in settings['arms'].items():
        if not isinstance(arm, dict) or set(arm) - {'hand', 'workspace'} or 'hand' not in arm:
            raise ValueError(f'arms.{profile} takes hand and, optionally, workspace')
        if arm['hand'] not in input_module.SIDES:
            raise ValueError(f"arms.{profile}.hand '{arm['hand']}' is not one of {list(input_module.SIDES)}")
        workspace = arm.get('workspace') or {}
        if set(workspace) - {'min', 'max'}:
            raise ValueError(f'arms.{profile}.workspace takes min and max')
        arms[profile] = ArmSettings(arm['hand'], ClutchConfig(
            xr_to_base=rotation,
            position_scale=_number(mapping, 'position_scale'),
            follow_rotation=bool(mapping['follow_rotation']),
            engage_threshold=_number(clutch, 'engage_threshold'),
            release_threshold=_number(clutch, 'release_threshold'),
            max_input_step=_number(clutch, 'max_input_step'),
            max_input_rotation_step=_number(clutch, 'max_input_rotation_step'),
            workspace_min=workspace.get('min'),
            workspace_max=workspace.get('max')))
    config = BridgeConfig(
        rate_hz=_positive(settings, 'rate_hz'),
        input_topic=str(source['topic']),
        input_timeout_sec=_positive(source, 'timeout_sec'),
        pose_source=str(source['pose_source']),
        button_threshold=_number(source, 'button_threshold'),
        run_toggle=buttons['run_toggle'],
        kill=buttons['kill'],
        stop_after_input_lost_sec=_positive(settings, 'stop_after_input_lost_sec'),
        stream_timeout_sec=_positive(stream, 'stream_timeout_sec'),
        horizon_steps=horizon,
        gripper_enabled=bool(settings['gripper']['enabled']),
        arms=arms)
    if config.pose_source not in input_module.POSE_SOURCES:
        raise ValueError(f"input.pose_source '{config.pose_source}' is not one of {list(input_module.POSE_SOURCES)}")
    if config.run_toggle == config.kill:
        raise ValueError('buttons.run_toggle and buttons.kill must differ')
    return config


def _number(mapping, key):
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{key} must be a finite number, got {value!r}')
    return float(value)


def _positive(mapping, key):
    value = _number(mapping, key)
    if value <= 0.0:
        raise ValueError(f'{key} must be > 0, got {value}')
    return value
