"""
turn.py
=======

Defines classes for tracking turn-by-turn state and drone reservations
during simulation scheduling.
"""

from typing import Dict, List, Optional
from dataclasses import dataclass
from drone import Drone


@dataclass
class DroneStatus:
    """Represents the operational status of a single drone during a simulation turn.

    Attributes:
        drone: The drone instance being tracked.
        will_move: True if the drone moves to a new region/zone in this turn.
        on_transit: True if the drone is currently in multi-turn transit
            (e.g. entering a RESTRICTED zone).
    """

    drone: Drone
    will_move: bool = False
    on_transit: bool = False


class Turn:
    """Stores all drone status reservations for a single simulation step/turn.

    Attributes:
        reservations: Mapping from location identifier (connection name e.g. "A-B"
            or zone name) to a list of :class:`DroneStatus` objects occupying or
            traversing that location.
    """

    def __init__(self) -> None:
        """Initializes a new empty Turn with no reservations."""
        self.reservations: Dict[str, List[DroneStatus]] = {}

