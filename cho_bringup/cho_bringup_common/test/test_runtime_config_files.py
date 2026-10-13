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

"""Files a bringup hands to a vendor launch by name, which must be installed where it looks.

The real UR bringup includes ur_robot_driver's ur_control.launch.py with
``runtime_config_package:=cho_bringup_ur``. That launch loads
``<runtime_config_package>/config/<ur_type>_update_rate.yaml`` into the driver's
controller_manager node, before the controllers file, so the file has to exist in this
package's installed share and agree with the update_rate of ``real/controllers.yaml``
(the controllers the real bringup spawns are written for that rate).

Needs cho_bringup_ur installed; skipped when it is not, unless CHO_LAUNCH_GOLDEN_REQUIRE
names it (CI does, see test_launch_golden.py).
"""

import os
import re

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
GOLDEN_REAL_UR = os.path.join(HERE, 'launch_golden', 'expected', 'cho_bringup_ur', 'bringup_real_robot')


def load(path):
    with open(path) as stream:
        return yaml.safe_load(stream)


def update_rate(path):
    return load(path)['controller_manager']['ros__parameters']['update_rate']


@pytest.fixture(scope='module')
def ur_share():
    try:
        return get_package_share_directory('cho_bringup_ur')
    except PackageNotFoundError:
        if 'cho_bringup_ur' in os.environ.get('CHO_LAUNCH_GOLDEN_REQUIRE', '').split():
            pytest.fail('cho_bringup_ur is not installed, but CHO_LAUNCH_GOLDEN_REQUIRE names it')
        pytest.skip('cho_bringup_ur is not installed')


def real_driver_includes():
    """(runtime_config_package, ur_type, controllers_file) the real UR bringup gives the driver launch."""
    found = set()
    for name in sorted(os.listdir(GOLDEN_REAL_UR)):
        with open(os.path.join(GOLDEN_REAL_UR, name)) as stream:
            text = stream.read()
        for block in re.split(r'^ *INCLUDE ', text, flags=re.MULTILINE)[1:]:
            if not block.startswith('<share:ur_robot_driver>/launch/ur_control.launch.py'):
                continue
            args = dict(re.findall(r"^ *arg (\w+)='([^']*)'$", block, flags=re.MULTILINE))
            found.add((args['runtime_config_package'], args['ur_type'], args['controllers_file']))
    return sorted(found)


def test_the_real_bringup_names_one_driver_config_for_the_files_tested_here():
    assert real_driver_includes() == [('cho_bringup_ur', 'ur5e', 'real/controllers.yaml')], \
        'the real UR bringup now includes the driver launch with other arguments: ' \
        'test the update-rate file it looks for (see the module docstring)'


def test_the_ur_update_rate_file_is_installed(ur_share):
    path = os.path.join(ur_share, 'config', 'ur5e_update_rate.yaml')
    assert os.path.isfile(path), \
        f'{path} is missing: ur_control.launch.py looks it up in the runtime_config_package'
    # The upstream file's shape: nothing but the controller_manager's update_rate.
    assert load(path) == {'controller_manager': {'ros__parameters': {'update_rate': update_rate(path)}}}


def test_the_ur_update_rate_equals_the_real_controllers_update_rate(ur_share):
    rate = update_rate(os.path.join(ur_share, 'config', 'ur5e_update_rate.yaml'))
    assert isinstance(rate, (int, float)) and rate > 0
    assert rate == update_rate(os.path.join(ur_share, 'config', 'real', 'controllers.yaml'))
