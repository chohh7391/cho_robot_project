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

"""The aggregation window of one object, measured on the image clock. No ROS here.

``node.py`` keeps the last ``window_sec`` of detections per object and publishes
a pose from them. The window is measured in the CAMERAS' time, the image stamp,
so it does not care how late a frame reaches the node -- and that is also what
makes it blind to the stamps going backwards. A bag replayed in a loop, a
simulator restarted, a driver that re-zeroes its clock: the new stamps are all
EARLIER than what the window holds, so trimming "everything older than the
newest stamp minus ``window_sec``" trims nothing, the old samples sit in the
window next to the new ones, and the node publishes the old object position
until the new run's time catches up with the old one's.

**A rewind is judged per source, against that source's own newest stamp.** A
*source* is whatever stamps the samples -- the node passes the camera's name.
Two cameras on different clocks are not rewinding each other: camera B stamping
five seconds ahead of camera A is a steady offset, and comparing A's stamps with
B's newest would read every A frame as a jump back and empty the window on every
message, so nothing would ever be published. A source whose own stamps step back
by more than ``rewind_threshold_sec`` has rewound; smaller steps are not rewinds.
Two sources are never frame-synchronised and a message queue reorders a little,
so a sample a few tens of milliseconds late goes into its place in time order,
or is dropped when it is already older than the window.

**One source's rewind empties the whole window, not just that source's samples.**
The two events that rewind a source are a simulator restart or a looped bag,
where every camera jumps back together, and one driver restarting. In the first
the other cameras' samples are on the old clock too, and the first camera to
report the jump does so up to a frame period before the others do: keeping
theirs would publish the old object position for that frame period, which is
the bug this exists to fix. In the second the others' samples are valid, and
emptying them costs one ``window_sec`` of refilling. A pose handed to a robot as
a target is worse stale than late, so the window takes the second cost to be
safe in the first case. The other sources keep their own newest stamp, so each
one that jumped reports its own rewind when it next speaks, which empties the
window once more and loses one sample from the sources that already spoke.

The window itself is still pruned against the newest stamp of any source, so a
source whose clock lags the leader's by more than ``window_sec`` contributes
nothing: one window cannot hold two time bases.
"""

import math

#: A stamp this far before its source's newest one is a clock rewind, not a late
#: frame. The same figure the node uses for how far its own clock may sit from
#: the detections' (``node._MAX_CLOCK_SKEW_SEC``): two clocks more than a second
#: apart are not one clock, whichever pair of them it is.
DEFAULT_REWIND_THRESHOLD_SEC = 1.0


class SampleWindow:
    """The samples of the last ``window_sec`` of image time, oldest first.

    Not thread-safe; ``node.py`` calls it with its aggregation lock held.
    """

    def __init__(self, window_sec, rewind_threshold_sec=DEFAULT_REWIND_THRESHOLD_SEC):
        window_sec = float(window_sec)
        rewind_threshold_sec = float(rewind_threshold_sec)
        if not math.isfinite(window_sec) or window_sec < 0.0:
            raise ValueError(f'window_sec must be finite and >= 0, got {window_sec}')
        if not math.isfinite(rewind_threshold_sec) or rewind_threshold_sec <= 0.0:
            raise ValueError(
                f'rewind_threshold_sec must be finite and > 0, got {rewind_threshold_sec}')
        self._window = window_sec
        self._rewind_threshold = rewind_threshold_sec
        self._entries = []
        # The newest stamp this window has been told about, by a sample or by
        # advance(), from any source: what the window is pruned against. Both
        # count: an object that is out of view still has to age its samples
        # out, and notice the clock jumping, on the other objects' detections.
        self._newest = None
        # The same, per source: what a rewind is judged against.
        self._newest_by_source = {}
        #: How far back the last rewind went, in seconds; 0.0 until one happens.
        self.last_rewind_sec = 0.0

    def add(self, stamp_sec, value, source=None):
        """Put *value* in the window at *stamp_sec*; True when a rewind cleared it first.

        *source* names who stamped it (the camera); a rewind is a step back of
        that source's own stamps. Leaving it out makes every sample one source.

        The sample is kept in time order, so one that arrives out of order
        (a slower camera, a reordered queue) sits where it belongs. A sample
        already older than the window is dropped.
        """
        stamp_sec = self._checked(stamp_sec)
        rewound = self._observe(stamp_sec, source)
        index = len(self._entries)
        while index > 0 and self._entries[index - 1][0] > stamp_sec:
            index -= 1
        self._entries.insert(index, (stamp_sec, value))
        self._prune()
        return rewound

    def advance(self, stamp_sec, source=None):
        """Tell the window *source*'s clock has reached *stamp_sec*; True on a rewind.

        Call it for every object on every detection message, whether or not that
        object was seen in it: that is what ages a quiet object's samples out
        and what clears a quiet object's window when the clock jumps back.
        """
        return self._observe(self._checked(stamp_sec), source)

    def samples(self):
        """Return ``[(stamp_sec, value), ...]``, time-ordered, within the window of the newest stamp."""
        return list(self._entries)

    def clear(self):
        """Forget every sample and every newest stamp; the next stamp starts afresh."""
        self._entries.clear()
        self._newest = None
        self._newest_by_source.clear()

    def __len__(self):
        return len(self._entries)

    @staticmethod
    def _checked(stamp_sec):
        stamp_sec = float(stamp_sec)
        if not math.isfinite(stamp_sec):
            raise ValueError(f'stamp must be finite, got {stamp_sec}')
        return stamp_sec

    def _observe(self, stamp_sec, source):
        source_newest = self._newest_by_source.get(source)
        if source_newest is not None and stamp_sec < source_newest - self._rewind_threshold:
            self.last_rewind_sec = source_newest - stamp_sec
            # Every source's samples go (see the module docstring), and the
            # prune reference restarts at the stamp that showed the jump. The
            # OTHER sources keep their newest stamp: each reports its own
            # rewind when it speaks, if it jumped too.
            self._entries.clear()
            self._newest = stamp_sec
            self._newest_by_source[source] = stamp_sec
            return True
        if source_newest is None or stamp_sec > source_newest:
            self._newest_by_source[source] = stamp_sec
        if self._newest is None or stamp_sec > self._newest:
            self._newest = stamp_sec
        self._prune()
        return False

    def _prune(self):
        # Strictly older than the window, as the node has always trimmed it: a
        # sample exactly window_sec old is still in.
        drop = 0
        while (drop < len(self._entries)
               and self._newest - self._entries[drop][0] > self._window):
            drop += 1
        if drop:
            del self._entries[:drop]
