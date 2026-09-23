"""
Decide, frame by frame, whether material is falling through a band of image.

No ROS and no clock of its own -- time arrives as a plain double -- so the whole
thing is testable against synthetic frames. It is also deliberately small: a
frame difference, a threshold, two counters. The hard part of this problem is
not the algorithm, it is where the camera points and what is behind the stream,
and no amount of cleverness here substitutes for a matte backdrop.

Three decisions are worth knowing before changing anything:

**The background is frozen while the stream is up.** A running average that
keeps updating during a pour absorbs the stream into the background within a
second or two, after which the detector reports that a pour in full flow has
stopped. The model updates only on frames where nothing stream-like was seen.

**A stream must cross the band, not merely change it.** Splashes, shadows,
reflections off a tilting vessel and the occasional insect all change pixels.
Material in free fall crosses every row of the band, so the row coverage is
what separates them, and it is a far better discriminator than a pixel count.

**Too much change is not a very good detection.** When the gripper swings
through the band, or someone switches a light on, most of the band changes at
once. That is reported as *not valid* rather than as a torrent: a consumer must
read invalid as "no information", never as "not flowing".
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Band:
    """
    The detection band, in NORMALIZED image coordinates (0..1 of width/height).

    Normalized rather than pixels so the band survives a resolution change: the
    OAK's mono streams can be reconfigured, and a band in pixels silently ends
    up pointing at the gripper when they are.
    """

    x0: float = 0.1
    y0: float = 0.45
    x1: float = 0.9
    y1: float = 0.60

    def slice_for(self, height: int, width: int) -> tuple[slice, slice]:
        """Return the (rows, cols) slice this band selects in an image that size."""
        r0 = int(round(min(self.y0, self.y1) * height))
        r1 = int(round(max(self.y0, self.y1) * height))
        c0 = int(round(min(self.x0, self.x1) * width))
        c1 = int(round(max(self.x0, self.x1) * width))
        r0 = max(0, min(r0, height - 1))
        c0 = max(0, min(c0, width - 1))
        r1 = max(r0 + 1, min(r1, height))
        c1 = max(c0 + 1, min(c1, width))
        return slice(r0, r1), slice(c0, c1)

    def validate(self) -> str:
        """Return an empty string when usable, or why it is not."""
        for name, value in (('x0', self.x0), ('y0', self.y0), ('x1', self.x1), ('y1', self.y1)):
            if not 0.0 <= value <= 1.0:
                return f'band.{name} must be a normalized 0..1 coordinate (got {value})'
        if abs(self.x1 - self.x0) < 0.02:
            return 'band is narrower than 2% of the image; it has to span the arc the lip travels'
        if abs(self.y1 - self.y0) < 0.01:
            return 'band has no height'
        if abs(self.y1 - self.y0) > 0.5:
            return ('band is taller than half the image; keep it inside the free-fall gap, clear '
                    'of the gripper above and the receiving rim below')
        return ''


@dataclass
class DetectorConfig:
    band: Band = field(default_factory=Band)
    #: Intensity change that counts as a changed pixel. Set it from what the
    #: tuner reports with nothing pouring: comfortably above the noise floor.
    diff_threshold: int = 12
    #: Changed pixels needed before a frame is even considered. Guards against
    #: single-pixel sensor noise; the row coverage below does the real work.
    min_pixels: int = 30
    #: Fraction of the band's rows the change must span. Material in free fall
    #: crosses all of them.
    min_row_coverage: float = 0.6
    #: Above this fraction of the band changed, it is an occlusion or a lighting
    #: change, not a stream.
    max_changed_fraction: float = 0.5
    #: Frames above threshold before flowing is asserted. Small on purpose: the
    #: rising edge is the pouring onset, and detecting it early is the entire
    #: reason for the camera.
    open_frames: int = 2
    #: Frames below threshold before flowing is released. Long enough to bridge
    #: sensor noise, NOT long enough to bridge the gap between drops -- this
    #: reports the instant, and deciding whether a pour is over belongs to
    #: whatever is counting the mass.
    close_frames: int = 12
    #: Frames spent learning the background before anything is reported.
    warmup_frames: int = 20
    #: Background blend rate on quiet frames.
    background_alpha: float = 0.02
    #: Consecutive unusable frames after which the background is re-seeded. What
    #: this recovers from is a light being switched: the old model is wrong
    #: forever otherwise.
    reset_after_invalid: int = 60

    def validate(self) -> str:
        why = self.band.validate()
        if why:
            return why
        if not 0 < self.diff_threshold < 255:
            return f'diff_threshold must be within 1..254 (got {self.diff_threshold})'
        if self.min_pixels < 1:
            return 'min_pixels must be at least 1'
        if not 0.0 < self.min_row_coverage <= 1.0:
            return f'min_row_coverage must be within 0..1 (got {self.min_row_coverage})'
        if not 0.0 < self.max_changed_fraction <= 1.0:
            return f'max_changed_fraction must be within 0..1 (got {self.max_changed_fraction})'
        if self.open_frames < 1 or self.close_frames < 1:
            return 'open_frames and close_frames must be at least 1'
        if self.warmup_frames < 1:
            return 'warmup_frames must be at least 1'
        if not 0.0 < self.background_alpha <= 1.0:
            return f'background_alpha must be within 0..1 (got {self.background_alpha})'
        return ''


@dataclass(frozen=True)
class StreamState:
    flowing: bool = False
    coverage: float = 0.0
    changed_pixels: int = 0
    valid: bool = False
    status: str = 'no frame yet'


class StreamDetector:
    """Frame-differencing stream presence over one band of the image."""

    def __init__(self, config: DetectorConfig):
        why = config.validate()
        if why:
            raise ValueError(why)
        self._config = config
        self._background: np.ndarray | None = None
        self._warmed = 0
        self._above = 0
        self._below = 0
        self._invalid_run = 0
        self._flowing = False

    @property
    def config(self) -> DetectorConfig:
        return self._config

    def reset(self) -> None:
        self._background = None
        self._warmed = 0
        self._above = 0
        self._below = 0
        self._invalid_run = 0
        self._flowing = False

    def update(self, gray: np.ndarray) -> StreamState:
        """Feed one greyscale frame and get the current verdict."""
        if gray.ndim != 2:
            return self._invalid('frame is not a single plane of intensities')

        rows, cols = self._config.band.slice_for(gray.shape[0], gray.shape[1])
        band = gray[rows, cols].astype(np.float32)

        if self._background is None or self._background.shape != band.shape:
            self._background = band.copy()
            self._warmed = 1
            return self._invalid('learning the background')

        diff = np.abs(band - self._background)
        mask = diff > self._config.diff_threshold
        changed = int(mask.sum())
        fraction = changed / float(mask.size)
        # A row the stream crosses has at least one changed pixel in it.
        coverage = float(mask.any(axis=1).mean())

        if self._warmed < self._config.warmup_frames:
            self._warmed += 1
            self._blend(band, alpha=0.25)
            return self._invalid('learning the background')

        if fraction > self._config.max_changed_fraction:
            self._invalid_run += 1
            if self._invalid_run >= self._config.reset_after_invalid:
                # Whatever changed is not going away. Adopt it as the new normal
                # so the detector recovers instead of reporting invalid forever.
                self._background = band.copy()
                self._warmed = 1
                self._invalid_run = 0
                return self._invalid('re-learning the background after a lasting change')
            return self._invalid(
                f'{fraction:.0%} of the band changed at once, which is an occlusion or a '
                'lighting change rather than a stream')
        self._invalid_run = 0

        candidate = (changed >= self._config.min_pixels and
                     coverage >= self._config.min_row_coverage)

        if candidate:
            self._above += 1
            self._below = 0
            if self._above >= self._config.open_frames:
                self._flowing = True
        else:
            self._below += 1
            self._above = 0
            if self._below >= self._config.close_frames:
                self._flowing = False
            # Only quiet frames teach the background. Updating it while the
            # stream is up dissolves the stream into it, and the detector then
            # reports that a pour in full flow has stopped.
            self._blend(band, alpha=self._config.background_alpha)

        return StreamState(flowing=self._flowing, coverage=coverage, changed_pixels=changed,
                           valid=True, status='')

    def _blend(self, band: np.ndarray, alpha: float) -> None:
        assert self._background is not None
        self._background *= (1.0 - alpha)
        self._background += alpha * band

    def _invalid(self, status: str) -> StreamState:
        # Never assert flowing from a frame that could not be read. A consumer
        # reads invalid as "no information"; asserting either way would make it
        # act on a guess.
        self._flowing = False
        self._above = 0
        self._below = 0
        return StreamState(flowing=False, coverage=0.0, changed_pixels=0, valid=False,
                           status=status)
