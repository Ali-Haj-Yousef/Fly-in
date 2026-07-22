"""
connection.py
=============

Defines the Connection data class representing network links between zones.
"""

from dataclasses import dataclass
from zone import Zone


@dataclass
class Connection:
    """Represents a bidirectional connection (edge) between two zones.

    Attributes:
        zone_a: First endpoint of the connection.
        zone_b: Second endpoint of the connection.
        max_link_capacity: Maximum number of drones allowed to traverse
            this connection simultaneously.
        blocked: Flag indicating whether this connection is marked blocked
            due to dead-end paths or blocked zones.
    """

    zone_a: Zone
    zone_b: Zone
    max_link_capacity: int = 1
    blocked: bool = False

    @property
    def name(self) -> str:
        """Canonical name of the connection, e.g. ``hub-roof1``."""
        return f"{self.zone_a.name}-{self.zone_b.name}"

    def __repr__(self) -> str:  # pragma: no cover - cosmetic only
        """Returns a concise developer-friendly string for this connection."""
        return (
            f"Connection({self.zone_a.name}<->{self.zone_b.name}, "
            f"capacity={self.max_link_capacity})"
        )
