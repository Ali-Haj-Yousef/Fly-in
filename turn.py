"""
turn.py
=======

Defines classes for tracking turn-by-turn state and drone reservations
during simulation scheduling.
"""

from dataclasses import dataclass
from typing import Dict, List

from drone import Drone


@dataclass
class DroneStatus:
    """Represent the operational status of one drone during a turn.

    Attributes:
        drone: The drone instance being tracked.
        will_move: True if the drone moves to a new region/zone in this turn.
        on_transit: True if the drone is currently in multi-turn transit
            (for example, entering a RESTRICTED zone).
    """

    drone: Drone
    will_move: bool = False
    on_transit: bool = False
    next_connection: str | None = None


class Turn:
    """Store all drone status reservations for one simulation step.

    Attributes:
        reservations: Mapping from location identifier (for example, a
            connection name like "A-B" or a zone name) to a list of
            :class:`DroneStatus` objects occupying or traversing that
            location.
    """

    def __init__(self) -> None:
        """Initializes a new empty Turn with no reservations."""
        self.reservations: Dict[str, List[DroneStatus]] = {}
