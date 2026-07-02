# drone.py
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional


class StepKind(Enum):
    WAIT = auto()
    MOVE = auto()
    ENTER_TRANSIT = auto()
    COMPLETE_TRANSIT = auto()


@dataclass(frozen=True)
class ScheduledStep:
    turn: int
    kind: StepKind
    zone: str
    connection: Optional[str] = None   # used only for ENTER_TRANSIT


class Drone:
    def __init__(self, drone_id: int, schedule: List[ScheduledStep]):
        if not schedule or schedule[0].turn != 0:
            raise ValueError("Schedule must start at turn 0")
        self.drone_id = drone_id
        self.schedule = schedule
        self.next_step_index = 0
        self.delivered = False
        self.current_zone = schedule[0].zone
        self._validate_schedule()

    def _validate_schedule(self):
        # Check that turns are consecutive
        # and restricted transitions are valid.
        for i, step in enumerate(self.schedule):
            if step.turn != i:
                raise ValueError(f"Missing turn {i}")
            if step.kind == StepKind.ENTER_TRANSIT:
                if i + 2 >= len(self.schedule):
                    raise ValueError(
                        "ENTER_TRANSIT must be followed by COMPLETE_TRANSIT")
                if self.schedule[i+2].kind != StepKind.COMPLETE_TRANSIT:
                    raise ValueError(
                        "After ENTER_TRANSIT, must have COMPLETE_TRANSIT two turns later")
                if self.schedule[i+1].kind != StepKind.WAIT:
                    raise ValueError("Intermediate turn must be WAIT")
                # Optionally check the zone names match etc.

    def current_step(self) -> Optional[ScheduledStep]:
        if self.next_step_index < len(self.schedule):
            return self.schedule[self.next_step_index]
        return None

    def advance(self):
        if self.delivered:
            return
        self.next_step_index += 1
        if self.next_step_index >= len(self.schedule):
            self.delivered = True
            self.current_zone = None
        else:
            step = self.schedule[self.next_step_index]
            if step.kind in (StepKind.MOVE, StepKind.COMPLETE_TRANSIT):
                self.current_zone = step.zone

    def output_token(self) -> Optional[str]:
        step = self.current_step()
        if step is None or self.delivered:
            return None
        if step.kind == StepKind.WAIT:
            return None
        if step.kind == StepKind.MOVE:
            return f"D{self.drone_id}-{step.zone}"
        if step.kind == StepKind.ENTER_TRANSIT:
            return f"D{self.drone_id}-{step.connection}"
        if step.kind == StepKind.COMPLETE_TRANSIT:
            return f"D{self.drone_id}-{step.zone}"
        return None
