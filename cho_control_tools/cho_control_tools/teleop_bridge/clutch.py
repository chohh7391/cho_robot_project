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

"""The clutch: a hand's motion since it engaged, applied to the pose the arm holds.

An absolute-pose device (an XR controller) and the arm live in different frames
and different places, so the bridge never sends the hand's pose itself. Each
arm holds a target in its absolute goal frame. A squeeze engages the clutch:
that instant latches the hand's pose ``(p_c0, R_c0)`` and the target
``(p_h, R_h)``, and from then on

    p = p_h + s · R_a (p_c − p_c0)
    R = (R_a · R_c R_c0ᵀ · R_aᵀ) · R_h

with ``R_a`` the XR-world-to-goal-frame rotation and ``s`` the position scale.
On the engaging frame ``p_c = p_c0``, so the target does not move: engaging, and
engaging again after the hand was moved with the clutch open, never jumps.
Released, the target stays where it was, which the stream keeps sending (the
arm holds).

The clutch engages only on a fresh press (a rising edge through
``engage_threshold``, released below ``release_threshold``), and lets go when
the press ends, the stream is disabled, the hand has no valid pose, or the
hand jumps further in one frame than ``max_input_step`` /
``max_input_rotation_step`` (a tracking glitch, not a motion); after any of
those the operator has to release and press again.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class ClutchConfig:
    """One arm's clutch. ``xr_to_base`` is the 3×3 rotation taking XR world axes to the goal frame's."""

    xr_to_base: np.ndarray
    position_scale: float = 1.0
    follow_rotation: bool = True
    engage_threshold: float = 0.6
    release_threshold: float = 0.4
    max_input_step: float = 0.1
    max_input_rotation_step: float = 0.5
    workspace_min: np.ndarray | None = None
    workspace_max: np.ndarray | None = None

    def __post_init__(self):
        matrix = np.asarray(self.xr_to_base, dtype=float)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError('xr_to_base must be a finite 3x3 matrix')
        if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-6) or np.linalg.det(matrix) < 0.0:
            raise ValueError('xr_to_base must be a rotation (orthonormal, determinant +1)')
        object.__setattr__(self, 'xr_to_base', matrix)
        if not (np.isfinite(self.position_scale) and self.position_scale > 0.0):
            raise ValueError('position_scale must be finite and > 0')
        if not 0.0 <= self.release_threshold < self.engage_threshold <= 1.0:
            raise ValueError('need 0 <= release_threshold < engage_threshold <= 1')
        if not (self.max_input_step > 0.0 and self.max_input_rotation_step > 0.0):
            raise ValueError('max_input_step and max_input_rotation_step must be > 0')
        if (self.workspace_min is None) != (self.workspace_max is None):
            raise ValueError('give both workspace_min and workspace_max, or neither')
        if self.workspace_min is not None:
            low = np.asarray(self.workspace_min, dtype=float)
            high = np.asarray(self.workspace_max, dtype=float)
            if low.shape != (3,) or high.shape != (3,) or not np.all(low < high):
                raise ValueError('workspace_min and workspace_max must be 3 values each, min < max')
            object.__setattr__(self, 'workspace_min', low)
            object.__setattr__(self, 'workspace_max', high)


class Clutch:
    """One arm's clutch; see the module docstring."""

    def __init__(self, config: ClutchConfig):
        self._config = config
        self._xr_to_base = Rotation.from_matrix(config.xr_to_base)
        self.reset()

    def reset(self):
        """Forget the target: nothing is held until the next ``seed``."""
        self._target = None
        self._engaged = False
        # A press already held when the stream (re)starts must be released first.
        self._pressed = True
        self._origin = None
        self._home = None
        self._last_input = None

    def seed(self, position, orientation):
        """Hold the arm at this pose (goal frame; quaternion x, y, z, w), disengaged."""
        position = np.asarray(position, dtype=float).reshape(3)
        orientation = np.asarray(orientation, dtype=float).reshape(4)
        if not (np.all(np.isfinite(position)) and np.all(np.isfinite(orientation))):
            raise ValueError('seed pose must be finite')
        self.reset()
        self._target = (position.copy(), Rotation.from_quat(orientation))

    @property
    def engaged(self) -> bool:
        return self._engaged

    @property
    def target(self):
        """``(position, quaternion xyzw)`` the arm should hold, or None before a seed."""
        if self._target is None:
            return None
        position, rotation = self._target
        return position.copy(), rotation.as_quat()

    def update(self, sample, enabled):
        """Advance one frame with *sample* (a HandSample, or None) and return the target.

        *enabled* is False while the stream must not follow the hand (stopped,
        input stale): the clutch then lets go as for a release.
        """
        squeeze = sample.squeeze if sample is not None else 0.0
        if self._pressed:
            pressed = squeeze > self._config.release_threshold
        else:
            pressed = squeeze >= self._config.engage_threshold
        rising = pressed and not self._pressed
        self._pressed = pressed

        valid = sample is not None and sample.valid
        if self._target is None or not (enabled and valid and pressed):
            self._let_go()
            return self.target

        hand_position = np.asarray(sample.position, dtype=float)
        hand_rotation = Rotation.from_quat(sample.orientation)
        if self._engaged and self._jumped(hand_position, hand_rotation):
            self._let_go()
            self._pressed = True  # release, then press again
            return self.target
        if not self._engaged:
            if not rising:
                return self.target
            self._engaged = True
            self._origin = (hand_position, hand_rotation)
            self._home = self._target
        self._last_input = (hand_position, hand_rotation)

        origin_position, origin_rotation = self._origin
        home_position, home_rotation = self._home
        position = home_position + self._config.position_scale * self._xr_to_base.apply(
            hand_position - origin_position)
        if self._config.follow_rotation:
            delta = self._xr_to_base * (hand_rotation * origin_rotation.inv()) * self._xr_to_base.inv()
            rotation = delta * home_rotation
        else:
            rotation = home_rotation
        if self._config.workspace_min is not None:
            position = np.clip(position, self._config.workspace_min, self._config.workspace_max)
        self._target = (position, rotation)
        return self.target

    def _let_go(self):
        self._engaged = False
        self._origin = None
        self._home = None
        self._last_input = None

    def _jumped(self, position, rotation):
        last_position, last_rotation = self._last_input
        step = float(np.linalg.norm(position - last_position))
        turn = float((rotation * last_rotation.inv()).magnitude())
        return step > self._config.max_input_step or turn > self._config.max_input_rotation_step
