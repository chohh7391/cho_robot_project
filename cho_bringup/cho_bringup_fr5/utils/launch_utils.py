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

"""Shared launch helpers for the FR5 bringups.

Installed to lib/, not as an importable package, and loaded by path the way
cho_bringup_franka and cho_bringup_openarm load theirs.
"""

# The gripper names fr5.ros2_control.xacro switches on.
GRIPPERS = ('none', 'ag95')
# The FR5's only gripper, so the boolean `load_gripper` spelling is unambiguous.
DEFAULT_GRIPPER = 'ag95'

_TRUE = ('true', '1', 'yes', 'on')
_FALSE = ('false', '0', 'no', 'off')
_DEFER = ('', 'config')


def as_bool(value):
    """Parse a launch argument that means true or false, and nothing else."""
    text = str(value).strip().lower()
    if text in _TRUE:
        return True
    if text in _FALSE:
        return False
    raise RuntimeError(
        f"Expected a boolean launch argument, got '{value}'. "
        f"Valid: {', '.join(_TRUE + _FALSE)}")


def resolve_gripper(gripper_arg, load_gripper_arg, config_value):
    """Resolve the `gripper` / `load_gripper` pair against the config file.

    `gripper` NAMES the gripper (none | ag95) and is the more specific form, so
    it stays the one the description and the hardware parameters are expressed
    in. `load_gripper` is the boolean spelling cho_bringup_franka uses for the
    same choice: true loads the FR5's only gripper, false loads none, and
    'config' - the default - defers to fr5.config.yaml.

    Passing both is allowed only when they agree. A contradictory pair is a
    launch error rather than one silently winning, because "I asked for no
    gripper and got one" is exactly the surprise worth refusing to deliver.
    """
    from_config = str(config_value if config_value is not None else 'none').strip()
    if from_config not in GRIPPERS:
        raise RuntimeError(
            f"Unknown gripper '{from_config}' in the config file. "
            f"Valid options: {', '.join(GRIPPERS)}")

    named = str(gripper_arg or '').strip()
    if named and named not in GRIPPERS:
        raise RuntimeError(
            f"Unknown gripper '{named}'. Valid options: {', '.join(GRIPPERS)}")

    load_text = str(load_gripper_arg if load_gripper_arg is not None else 'config').strip().lower()
    if load_text in _DEFER:
        from_load = None
    else:
        from_load = DEFAULT_GRIPPER if as_bool(load_text) else 'none'

    if named and from_load is not None and named != from_load:
        raise RuntimeError(
            f"gripper:={named} contradicts load_gripper:={load_text}. Pass one of them, "
            f"or make them agree (load_gripper true means gripper:={DEFAULT_GRIPPER}).")

    # An explicit argument beats the config file; either spelling counts.
    return named or from_load or from_config
