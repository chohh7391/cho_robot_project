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

"""Search-path environment variables, set by the launch rather than by Python."""

from launch.actions import AppendEnvironmentVariable


def prepend_to_search_paths(variables, path):
    """Actions that put `path` first on each of the os.pathsep lists in `variables`.

    Launch actions rather than an os.environ edit while the description is
    generated: they take effect in launch order, for the processes started
    after them, and do not leak into whatever else imports the launch file.
    A variable that is unset becomes `path` alone.
    """
    return [AppendEnvironmentVariable(name, path, prepend=True) for name in variables]
