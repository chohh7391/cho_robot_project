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

"""teleop_bridge's settings: the defaults parse, a project file overrides key by key, typos are refused."""

from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from cho_control_tools.teleop_bridge import config as CONFIG


# The input module's surface the settings check against, without importing msgpack.
DEVICE = SimpleNamespace(
    SIDES=('left', 'right'),
    POSE_SOURCES=('grip', 'aim'),
    button_names=lambda: tuple(f'{side}_{button}' for side in ('left', 'right')
                               for button in ('primary_click', 'secondary_click', 'thumbstick_click', 'menu_click')))
DEFAULT_ROTATION = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def write(tmp_path, settings):
    path = tmp_path / 'teleop.yaml'
    path.write_text(yaml.safe_dump({'teleop_bridge': settings}))
    return path


def test_the_defaults_parse():
    config = CONFIG.load_config(DEVICE, 'xr_controller')
    assert config.rate_hz == 60.0
    assert config.control_dt == pytest.approx(1.0 / 60.0)
    assert config.input_topic == '/xr_teleop/controller_data'
    assert (config.run_toggle, config.kill) == ('right_primary_click', 'right_secondary_click')
    assert config.horizon_steps == 3
    assert config.arm('left').hand == 'left'
    assert config.arm('single').hand == 'right'
    np.testing.assert_allclose(config.arm('right').clutch.xr_to_base, DEFAULT_ROTATION)


def test_a_project_file_changes_only_what_it_names(tmp_path):
    path = write(tmp_path, {
        'mapping': {'operator_yaw_deg': 180.0, 'position_scale': 0.5},
        'arms': {'left': {'hand': 'left', 'workspace': {'min': [0.0, 0.0, 0.0], 'max': [1.0, 1.0, 1.0]}}},
    })
    config = CONFIG.load_config(DEVICE, 'xr_controller', path)
    left = config.arm('left').clutch
    assert left.position_scale == 0.5
    np.testing.assert_allclose(
        left.xr_to_base, Rotation.from_euler('z', 180, degrees=True).as_matrix() @ DEFAULT_ROTATION, atol=1e-12)
    np.testing.assert_allclose(left.workspace_max, (1.0, 1.0, 1.0))
    assert config.arm('right').clutch.workspace_min is None
    assert config.rate_hz == 60.0
    assert config.arm('right').clutch.follow_rotation is True


@pytest.mark.parametrize('settings', [
    {'mapping': {'scale': 2.0}},
    {'rate': 30.0},
    {'buttons': {'run_toggle': 'right_trigger'}},
    {'buttons': {'kill': 'right_primary_click'}},
    {'arms': {'left': {'hand': 'middle'}}},
    {'arms': {'left': {'side': 'left'}}},
    {'stream': {'horizon_steps': 0}},
    {'rate_hz': -1.0},
    {'input': {'pose_source': 'palm'}},
    {'clutch': {'engage_threshold': 0.2}},
])
def test_bad_settings_are_refused(tmp_path, settings):
    with pytest.raises(ValueError):
        CONFIG.load_config(DEVICE, 'xr_controller', write(tmp_path, settings))


def test_an_arm_without_settings_names_the_key_to_add():
    config = CONFIG.load_config(DEVICE, 'xr_controller')
    with pytest.raises(ValueError, match='arms.both.hand'):
        config.arm('both')


def test_a_file_needs_the_teleop_bridge_key(tmp_path):
    path = tmp_path / 'teleop.yaml'
    path.write_text(yaml.safe_dump({'rate_hz': 30.0}))
    with pytest.raises(ValueError, match='teleop_bridge'):
        CONFIG.load_config(DEVICE, 'xr_controller', path)
