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
        weight: Cost used by shortest-path calculations.
        related_to_shortest_path: Whether this connection belongs to the
            currently computed shortest path.
    """

    zone_a: Zone
    zone_b: Zone
    max_link_capacity: int = 1
    related_to_shortest_path: bool = False
    blocked: bool = False

    @property
    def name(self) -> str:
        """Canonical name of the connection, e.g. ``hub-roof1``."""
        return f"{self.zone_a.name}-{self.zone_b.name}"

    def other_end(self, zone: Zone) -> Zone:
        """Given one endpoint, returns the opposite endpoint of the edge.

        Args:
            zone: One of the two zones forming this connection.

        Returns:
            Zone: The zone at the other end of the connection.

        Raises:
            ValueError: If ``zone`` is not one of this connection's endpoints.
        """
        if zone is self.zone_a:
            return self.zone_b
        if zone is self.zone_b:
            return self.zone_a
        raise ValueError(
            f"Zone {zone.name!r} is not part of connection {self.name!r}")

    def __repr__(self) -> str:  # pragma: no cover - cosmetic only
        """Returns a concise developer-friendly string for this connection."""
        return (
            f"Connection({self.zone_a.name}<->{self.zone_b.name}, "
            f"capacity={self.max_link_capacity})"
        )
