"""
graph.py
========

Defines the Graph container that manages network topology, zone mappings,
connection adjacencies, and graph analysis algorithms (for example,
blocking dead ends).
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
from zone import Zone, ZoneType
from connection import Connection


@dataclass
class Graph:
    """Container for the full drone network:
        zones, connections, and drone count.

    Attributes:
        nb_drones: Number of drones to route through the network.
        zones: Mapping of zone name to :class:`Zone` instance.
        connections: List of all :class:`Connection` instances.
        adjacency: Mapping of zone name to the list of connections touching it.
    """

    nb_drones: int = 0
    zones: Dict[str, Zone] = field(default_factory=dict)
    connections: List[Connection] = field(default_factory=list)
    adjacency: Dict[str, List[Connection]] = field(default_factory=dict)

    def block(self, zone: Zone, stack: list[Zone]) -> bool:
        """Recursively detect dead ends using DFS and mark inaccessible paths.

        Traverses from ``zone`` toward ``end_zone``. If a path from ``zone``
        cannot reach ``end_zone`` (or leads only to cycles or blocked zones),
        the connection and zone are marked as BLOCKED so the scheduler will
        not attempt to route drones through them.

        Args:
            zone: The current zone being evaluated.
            stack: Zones currently in the recursion stack to detect cycles.

        Returns:
            bool: True if the sub-path starting at ``zone`` cannot reach
                ``end_zone`` (is blocked).
        """
        # Base case: explicitly blocked zones cannot lead to destination
        if zone.zone_type == ZoneType.BLOCKED:
            return True
        blocked = True
        # Base case: reached destination hub
        if zone == self.end_zone:
            return False
        # Base case: cycle detected in stack
        if zone in stack:
            return True

        stack.append(zone)
        for connection in self.adjacency[zone.name]:
            if connection.blocked:
                continue
            next_zone = connection.zone_b
            if self.block(next_zone, stack):
                # Mark connection as blocked if it leads to a dead-end branch
                connection.blocked = True
            else:
                blocked = False
        stack.pop()

        # If all outgoing connections are blocked, mark this zone as BLOCKED
        if blocked:
            zone.zone_type = ZoneType.BLOCKED
        return blocked

    def add_zone(self, zone: Zone) -> None:
        """Registers a new zone in the graph.

        Args:
            zone: The zone to add.
        """
        self.zones[zone.name] = zone
        self.adjacency.setdefault(zone.name, [])

    def add_connection(self, connection: Connection) -> None:
        """Registers a new connection in the graph and updates adjacency.

        Args:
            connection: The connection to add.
        """
        self.connections.append(connection)
        self.adjacency[connection.zone_a.name].append(connection)

    @property
    def start_zone(self) -> Zone:
        """Returns the unique start zone.

        Raises:
            ParserError: If no start zone is registered (should not happen
                after a successful parse).
        """
        for zone in self.zones.values():
            if zone.is_start:
                return zone
        from parser import ParserError

        raise ParserError(0, "No start_hub zone found in graph")

    @property
    def end_zone(self) -> Zone:
        """Returns the unique end zone.

        Raises:
            ParserError: If no end zone is registered (should not happen
                after a successful parse).
        """
        for zone in self.zones.values():
            if zone.is_end:
                return zone
        from parser import ParserError

        raise ParserError(0, "No end_hub zone found in graph")

    def neighbors(self, zone: Zone) -> List[Connection]:
        """Returns the list of connections attached to a given zone.

        Args:
            zone: The zone whose neighboring connections are requested.
        """
        return self.adjacency.get(zone.name, [])

    def connection(
        self, zone_a_name: str, zone_b_name: str
    ) -> Optional["Connection"]:
        """
        Find the connection (if any) between two zones.

        Since connections are bidirectional, either order of zone names works.

        Args:
            zone_a_name: Name of the first zone.
            zone_b_name: Name of the second zone.

        Returns:
            The Connection object if found, otherwise None.
        """
        for conn in self.connections:
            if (
                conn.zone_a.name == zone_a_name
                and conn.zone_b.name == zone_b_name
            ) or (
                conn.zone_a.name == zone_b_name
                and conn.zone_b.name == zone_a_name
            ):
                return conn
        return None

    def __repr__(self) -> str:  # pragma: no cover - cosmetic only
        """Returns a concise developer-friendly string for this graph."""
        return (
            f"Graph(nb_drones={self.nb_drones}, "
            f"zones={len(self.zones)}, connections={len(self.connections)})"
        )
