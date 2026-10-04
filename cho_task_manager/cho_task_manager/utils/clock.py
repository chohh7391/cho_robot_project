"""Deadlines on the node clock that do not start before the clock does.

Under ``use_sim_time`` the node clock reads 0 until the first /clock message.
A deadline taken from that reading -- ``now() + 30 s`` -- is already far in the
past once /clock arrives with the simulator's real time, so the first tick
after it would call the deadline missed: a goal times out, a wait ends, a
sweep gives up, none of them having waited at all. Every deadline in the
behaviours is therefore taken through :func:`deadline_after`, which returns
None while the clock has not started, and is armed on the first tick that
sees a running clock (:func:`arm`).
"""

from rclpy.duration import Duration


def clock_started(now):
    """False for a node clock still at 0: sim time before the first /clock."""
    return now.nanoseconds != 0


def deadline_after(clock, seconds):
    """``clock.now() + seconds``, or None while *clock* has not started."""
    now = clock.now()
    if not clock_started(now):
        return None
    return now + Duration(seconds=float(seconds))


def arm(deadline, clock, seconds):
    """*deadline* if it is set, else one taken now -- still None if the clock has not started.

    For the tick-side check: ``self._deadline = arm(self._deadline, clock, s)``
    and then compare only when it is not None.
    """
    if deadline is not None:
        return deadline
    return deadline_after(clock, seconds)


#: The stamp of an event seen while the clock still read 0. When it happened
#: on the simulator's clock is unknown, so it is taken as "just now" at the
#: first reading that is not 0 (:func:`restamp`), never as the time since 0.
CLOCK_NOT_STARTED = 'clock-not-started'


def stamp(now):
    """*now*, or :data:`CLOCK_NOT_STARTED` while the clock reads 0."""
    return now if clock_started(now) else CLOCK_NOT_STARTED


def restamp(since, now):
    """*since*, re-taken as *now* if it was stamped before the clock started and it now runs."""
    if since is CLOCK_NOT_STARTED and clock_started(now):
        return now
    return since


def seconds_since(since, now):
    """Seconds from *since* to *now*; 0 for an event stamped before the clock started."""
    if since is CLOCK_NOT_STARTED or not clock_started(now):
        return 0.0
    return (now - since).nanoseconds * 1e-9
