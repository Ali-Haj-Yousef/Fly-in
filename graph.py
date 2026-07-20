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

    def block(self, zone: Zone, stack: list[Zone]):
        blocked = True
        if zone == self.end_zone:
            return False
        if zone.zone_type == ZoneType.BLOCKED or zone in stack:
            return True
        stack.append(zone)
        for connection in self.adjacency[zone.name]:
            if connection.blocked:
                continue
            # print(f"{zone.name}: {connection.name}")
            next_zone = connection.zone_b
            if self.block(next_zone, stack):
                connection.blocked = True
                print(f"{connection.name} is blocked")
            else:
                blocked = False
        stack.pop()
        # print(f"{zone.name}: blocked = {blocked}")
        # print()
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
        # opposite_connection = Connection(
        #     connection.zone_b,
        #     connection.zone_a,
        #     connection.max_link_capacity
        # )
        # self.adjacency[connection.zone_b.name].append(opposite_connection)

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
                conn.zone_a.name == zone_a_name and
                conn.zone_b.name == zone_b_name) or \
               (conn.zone_a.name == zone_b_name and
                    conn.zone_b.name == zone_a_name):
                return conn
        return None

    def __repr__(self) -> str:  # pragma: no cover - cosmetic only
        """Returns a concise developer-friendly string for this graph."""
        return (
            f"Graph(nb_drones={self.nb_drones}, "
            f"zones={len(self.zones)}, connections={len(self.connections)})"
        )

    @property
    def shortest_path(self) -> List[Zone]:
        """
        Find the shortest path from start_zone to end_zone using Dijkstra's
        algorithm.

        Args:
            start_zone: The starting zone.
            end_zone: The destination zone.

        Returns:
            A list of zones representing the shortest path from start_zone to
            end_zone. If no path exists, returns an empty list.
        """
        import heapq

        start_zone = self.start_zone
        end_zone = self.end_zone
        # Priority queue for Dijkstra's algorithm
        queue = [
            (0, 0 if start_zone.zone_type == ZoneType.PRIORITY else 1,
             start_zone.name, start_zone)
        ]
        distances = {zone.name: float('inf') for zone in self.zones.values()}
        previous_zones = {zone.name: None for zone in self.zones.values()}
        distances[start_zone.name] = 0

        while queue:
            current_distance, _, _, current_zone = heapq.heappop(queue)
            if current_distance > distances[current_zone.name]:
                continue

            for connection in self.neighbors(current_zone):
                neighbor = connection.zone_b

                distance = current_distance + neighbor.movement_cost
                should_update = (
                    distance < distances[neighbor.name] or
                    (
                        distance == distances[neighbor.name] and
                        neighbor.zone_type == ZoneType.PRIORITY
                    )
                )

                if should_update:
                    distances[neighbor.name] = distance
                    previous_zones[neighbor.name] = current_zone
                    heapq.heappush(
                        queue,
                        (
                            distance,
                            (0 if neighbor.zone_type == ZoneType.PRIORITY
                             else 1),
                            neighbor.name,
                            neighbor,
                        ),
                    )

        # Reconstruct the shortest path
        path = []
        current = end_zone
        while current is not None:
            path.append(current)
            current = previous_zones[current.name]

        path.reverse()

        if not path or path[0] != start_zone:
            return []  # No path found

        for index in range(len(path) - 1):
            connection = self.connection(
                path[index].name, path[index + 1].name
            )
            if connection is not None:
                connection.related_to_shortest_path = True

        return path
