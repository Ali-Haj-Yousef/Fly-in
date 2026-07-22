"""
zone.py
=======

Defines Zone entities, ZoneType enums, and HubRole enums that model
the spatial attributes, capacities, and costs of network nodes.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Optional


class ZoneType(Enum):
    """Defines the behavior and movement cost associated with a zone.

    Each member stores a ``(label, cost)`` tuple so that the movement cost
    lives directly alongside the type it describes.  ``label`` is the
    human-readable string used in map files and preserved as ``.value``.
    ``cost`` is ``-1`` for inaccessible zones (BLOCKED).

    Example:
        >>> ZoneType.RESTRICTED.value
        'restricted'
        >>> ZoneType.RESTRICTED.movement_cost
        2
    """

    NORMAL = ("normal", 1)
    BLOCKED = ("blocked", -1)
    RESTRICTED = ("restricted", 2)
    PRIORITY = ("priority", 1)

    def __new__(cls, label: str, cost: int) -> "ZoneType":
        """Constructs a ZoneType member, binding label and cost together.

        Args:
            label: The string identifier used in map files (e.g. ``"normal"``).
                   Stored as ``.value`` so existing
                   parser lookups are unchanged.
            cost:  Movement cost in simulation turns.  ``-1`` signals that the
                   zone is inaccessible and must never appear in a valid path.
        """
        obj = object.__new__(cls)
        obj._value_ = label   # keeps .value as the readable string
        obj._cost = cost  # type: ignore[attr-defined]
        # set once at class-definition time
        return obj

    @property
    def movement_cost(self) -> int:
        """Returns the simulation turns required to enter this zone.

        Returns:
            int: 1 for NORMAL and PRIORITY, 2 for RESTRICTED, -1 for BLOCKED.
        """
        return self._cost  # type: ignore[attr-defined]


class HubRole(Enum):
    """Marks the special role (if any) a zone plays in the network."""

    START = "start_hub"
    END = "end_hub"
    REGULAR = "hub"


@dataclass
class Zone:
    """Represents a single zone (node) in the drone network.

    Attributes:
        name: Unique identifier of the zone.
        x: Integer x-coordinate, used for visual representation.
        y: Integer y-coordinate, used for visual representation.
        zone_type: Behavior/cost category of the zone.
        color: Optional color hint for terminal/graphical display.
        max_drones: Maximum number of drones allowed simultaneously
            in this zone (ignored for start/end zones, which are unlimited).
        role: Whether this zone is the start, the end, or a regular hub.
    """

    name: str
    x: int
    y: int
    zone_type: ZoneType = ZoneType.NORMAL
    color: Optional[str] = None
    max_drones: int = 1
    role: HubRole = HubRole.REGULAR

    @property
    def is_start(self) -> bool:
        """Returns True if this zone is the unique start hub."""
        return self.role is HubRole.START

    @property
    def is_end(self) -> bool:
        """Returns True if this zone is the unique end hub."""
        return self.role is HubRole.END

    @property
    def movement_cost(self) -> int:
        """Returns the turn cost to move into this zone."""
        return self.zone_type.movement_cost

    def __repr__(self) -> str:  # pragma: no cover - cosmetic only
        """Returns a concise developer-friendly string for this zone."""
        return (
            f"Zone(name={self.name!r}, pos=({self.x}, {self.y}), "
            f"type={self.zone_type.value}, max_drones={self.max_drones}, "
            f"role={self.role.value})"
        )
