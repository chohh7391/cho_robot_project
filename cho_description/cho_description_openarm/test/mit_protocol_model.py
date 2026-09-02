"""Executable draft model for the OpenArm MIT lifecycle protocol.

This is deliberately not a backend.  It makes the design state machine testable
before any simulator actuator or real CAN plugin is changed.
"""

from dataclasses import dataclass
from enum import Enum
from math import isfinite


class Status(Enum):
    SAFE = 0
    ACTIVE = 1
    SAFE_TRANSITION = 2
    STALE = 3
    INVALID = 4
    FAULT = 5
    DISABLED = 6


@dataclass(frozen=True)
class TupleCommand:
    session_echo: int
    generation: int
    lease_cycles: int
    q_des: tuple
    dq_des: tuple
    kp: tuple
    kd: tuple
    tau_ff: tuple


class ConsumerModel:
    def __init__(self, dof=7, session_id=1, max_lease_cycles=20):
        self.dof = dof
        self.session_id = session_id
        self.max_lease_cycles = max_lease_cycles
        self.ack_generation = 0
        self.age = 0
        self.accepted_lease_cycles = 0
        self.status = Status.SAFE
        self.safe_generation = 0
        self.safe_ack_generation = 0
        self.switch_latched = False
        self.fault_latched = False

    def _valid(self, cmd):
        fields = (cmd.q_des, cmd.dq_des, cmd.kp, cmd.kd, cmd.tau_ff)
        return (
            not self.fault_latched
            and not self.switch_latched
            and cmd.session_echo == self.session_id
            and self.ack_generation < cmd.generation <= 2**53 - 1
            and 0 < cmd.lease_cycles <= self.max_lease_cycles
            and all(len(field) == self.dof for field in fields)
            and all(isfinite(value) for field in fields for value in field)
            and all(value >= 0 for value in cmd.kp + cmd.kd)
        )

    def accept(self, cmd):
        if self.fault_latched or self.switch_latched or not self._valid(cmd):
            if not self.fault_latched and not self.switch_latched:
                self.status = Status.INVALID
            return False
        self.ack_generation = cmd.generation
        self.age = 0
        self.accepted_lease_cycles = cmd.lease_cycles
        self.status = Status.ACTIVE
        return True

    def write_cycle_without_commit(self):
        if self.status is Status.ACTIVE:
            self.age += 1
            if self.age >= self.accepted_lease_cycles:
                self.status = Status.STALE

    def prepare_external_stop(self):
        self.switch_latched = True
        self.safe_generation += 1
        self.status = Status.SAFE_TRANSITION

    def submit_hardware_safe(self):
        if self.status is not Status.SAFE_TRANSITION:
            return False
        self.safe_ack_generation = self.safe_generation
        self.status = Status.SAFE
        return True

    def hardware_fault(self):
        self.fault_latched = True
        self.status = Status.FAULT

    def deactivate(self):
        self.switch_latched = True
        self.status = Status.DISABLED


class BimanualConsumerModel:
    """One hardware component owns both arms; acceptance is all-or-none."""

    def __init__(self, session_id=1, max_lease_cycles=20):
        self.left = ConsumerModel(session_id=session_id, max_lease_cycles=max_lease_cycles)
        self.right = ConsumerModel(session_id=session_id, max_lease_cycles=max_lease_cycles)

    def accept_both(self, left, right):
        if left.generation != right.generation:
            return False
        # Preflight BOTH sides, including session and latches, before mutating
        # either ack.  The assignment below models a shadow-buffer commit.
        if not self.left._valid(left) or not self.right._valid(right):
            return False
        generation = left.generation
        self.left.ack_generation = generation
        self.right.ack_generation = generation
        self.left.accepted_lease_cycles = left.lease_cycles
        self.right.accepted_lease_cycles = right.lease_cycles
        self.left.age = self.right.age = 0
        self.left.status = self.right.status = Status.ACTIVE
        return True

    def fault_arm(self, side, both_arms_session=False):
        faulty = self.left if side == "left" else self.right
        peer = self.right if side == "left" else self.left
        faulty.hardware_fault()
        if both_arms_session:
            peer.hardware_fault()
        elif peer.status is Status.ACTIVE:
            peer.prepare_external_stop()  # configurable peer policy; default controlled hold
