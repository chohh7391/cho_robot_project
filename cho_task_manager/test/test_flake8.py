# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import os

from ament_flake8.main import main_with_errors
import pytest

# Paths are resolved from this file rather than from the working directory, as
# cho_object_pose's test does: `colcon test` runs pytest with the build
# directory as cwd, where a --symlink-install tree only partly mirrors the
# sources. And the paths come BEFORE --exclude, which takes any number of
# values: listed after it, they were all swallowed as excludes and only
# scripts/ and meshes/ were ever checked.
PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGETS = [os.path.join(PACKAGE_ROOT, name)
           for name in ('cho_task_manager', 'launch', 'setup.py', 'test')]


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[
        '--config=%s' % os.path.join(PACKAGE_ROOT, 'setup.cfg'),
        '--linelength=120',
    ] + TARGETS + [
        '--exclude',
        'python',
    ])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
