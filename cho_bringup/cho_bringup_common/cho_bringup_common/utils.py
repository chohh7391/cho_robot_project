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

"""Small helpers every bringup used to carry its own copy of."""

import importlib.util
import os

import yaml


def load_yaml(file_path):
    """Load a YAML file, failing with the path when it does not exist."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f'File not found: {file_path}')
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def as_bool(value):
    """Read a launch argument as a boolean, leniently.

    true / 1 / yes / on (any case) are True and everything else is False, so a
    typo reads as false rather than failing. Where a typo must fail instead,
    use a strict parser (cho_bringup_fr5's launch_utils.strict_bool).
    """
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('true', '1', 'yes', 'on')


def unique_names(names):
    """`names` without repeats, first occurrence kept, order preserved."""
    unique = []
    for name in names:
        if name not in unique:
            unique.append(name)
    return unique


def load_package_utils(package, module='launch_utils'):
    """Load lib/<package>/utils/<module>.py, a bringup's robot-specific helpers.

    Those modules are installed to lib/ rather than as importable packages, so
    they are loaded by path. The path is resolved from the package's share
    directory, which is what makes it follow whichever install space (or
    --symlink-install source tree) the package was found in.
    """
    from ament_index_python.packages import get_package_share_directory

    share = get_package_share_directory(package)
    path = os.path.abspath(os.path.join(share, '..', '..', 'lib', package, 'utils', f'{module}.py'))
    spec = importlib.util.spec_from_file_location(f'{package}_{module}', path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded
