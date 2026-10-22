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

"""A task's own parameters: one YAML file, ``task_params:=<path>``.

The node hands the file's mapping to the tree builder as
``robot_config['task_params']`` ({} without one), so a task registered by
another package takes parameters without a node or launch argument of its own.
"""

import yaml


def load_task_params(path):
    """The mapping in the YAML file at *path*; {} for an empty *path*.

    Raises ValueError for a file that cannot be read or parsed, or that holds
    anything but a mapping. An empty file is an empty mapping.
    """
    if not path:
        return {}
    try:
        with open(path, encoding='utf-8') as stream:
            params = yaml.safe_load(stream)
    except OSError as error:
        raise ValueError(f'task_params {path}: {error.strerror}') from error
    except yaml.YAMLError as error:
        raise ValueError(f'task_params {path} is not YAML: {error}') from error
    if params is None:
        return {}
    if not isinstance(params, dict):
        raise ValueError(f'task_params {path} holds a {type(params).__name__}, not a mapping')
    return params
