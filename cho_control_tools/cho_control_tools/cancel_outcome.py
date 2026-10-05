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

"""What a cancel request's answer actually says, in words an operator can act on.

``cancel_goal_async()`` completing means only that the server ANSWERED, and the
answer can be a refusal. ``action_msgs/srv/CancelGoal`` returns ``ERROR_NONE``
with the goals it is now cancelling, ``ERROR_REJECTED`` when it declines (the
goal runs on), and ``ERROR_UNKNOWN_GOAL_ID`` / ``ERROR_GOAL_TERMINATED`` when
there was nothing left to cancel. A client that prints "cancelled" for every
answer tells the operator the arm stopped when it may not have.
"""

from collections import namedtuple

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

# accepted: the server is now cancelling the goal (it is CANCELING, not yet
# CANCELED: the goal's result says how it ended). text: a lower-case clause.
CancelOutcome = namedtuple('CancelOutcome', ('accepted', 'text'))

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def describe_cancel_response(response):
    """The :class:`CancelOutcome` of a ``CancelGoal.Response``; ``None`` is no answer."""
    if response is None:
        return CancelOutcome(False, 'no answer to the cancel; the arm may still be moving')
    code = getattr(response, 'return_code', None)
    if code == CancelGoal.Response.ERROR_NONE:
        if list(getattr(response, 'goals_canceling', None) or []):
            return CancelOutcome(True, 'cancel accepted; the server is stopping the goal')
        # The contract says a cancel that touches no goal is ERROR_REJECTED,
        # but a server that answers this way has not said it is stopping ours.
        return CancelOutcome(
            False, 'the server answered the cancel but is cancelling no goal; '
                   'the arm may still be moving')
    if code == CancelGoal.Response.ERROR_REJECTED:
        return CancelOutcome(
            False, 'cancel REJECTED by the server; the goal is still running and '
                   'the arm may still be moving')
    if code == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID:
        return CancelOutcome(
            False, 'the server does not know this goal; nothing was cancelled')
    if code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
        return CancelOutcome(
            False, 'the goal had already finished; nothing was cancelled')
    return CancelOutcome(
        False, f'unrecognised answer to the cancel (return_code={code}); '
               'the arm may still be moving')


def goal_status_name(status):
    """``SUCCEEDED``, ``CANCELED``, ... for an ``action_msgs/GoalStatus`` value."""
    return GOAL_STATUS_NAMES.get(status, f'status {status}')


def sentence(text):
    """*text* with its first letter capitalised, for printing on its own."""
    return text[:1].upper() + text[1:]
