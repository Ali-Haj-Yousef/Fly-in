"""
scheduler.py
============

Implements the multi-drone path finding and
conflict-avoidance scheduling algorithm.

Routes drones greedy towards the destination hub (`end_hub`) while respecting:
- Max drone capacities per zone (`max_drones`)
- Max concurrent traverse capacities per connection (`max_link_capacity`)
- Movement cost differences across zone types
(NORMAL, RESTRICTED, PRIORITY, BLOCKED)
- Turn-by-turn step synchronization
"""

from collections import deque
from typing import Deque, Dict, List, Optional

from connection import Connection
from drone import Drone
from graph import Graph
from zone import Zone, ZoneType
from turn import DroneStatus, Turn


class Scheduler:
    """Schedules drone movements across turns to optimize
    throughput without collision or capacity breach.

    Attributes:
        graph: The drone network graph container.
        drones: List of drones to route from start_hub to end_hub.
        reservations: Turn-by-turn history of drone positions and movements.
        dist_to_end: Distance lookup map from each zone to the end hub.
    """

    def __init__(self, graph: Graph, drones: list[Drone]):
        """Initializes the Scheduler with network graph and drone fleet.

        Args:
            graph: The Graph network instance.
            drones: List of Drone objects to be scheduled.
        """
        self.graph = graph
        self.drones = drones
        self.reservations: List[Turn] = []
        self.dist_to_end: Dict[str, int] = {}

    def _compute_distances_to_end(self) -> Dict[str, int]:
        """Reverse BFS to compute the shortest hop distance to the end hub.

        Accounts for zone movement cost (e.g. extra cost for RESTRICTED zones).

        Returns:
            dict: Mapping of zone name to integer distance metric to end hub.
        """
        end = self.graph.end_zone
        dist: Dict[str, int] = {end.name: 0}
        queue: Deque[Zone] = deque([end])
        while queue:
            zone = queue.popleft()
            for conn in self.graph.connections:
                if conn.blocked:
                    continue
                if (
                    conn.zone_b.name == zone.name
                    and conn.zone_a.name not in dist
                ):
                    dist[conn.zone_a.name] = dist[zone.name] + 1
                    # Restricted zones cost an extra turn of delay
                    if zone.zone_type == ZoneType.RESTRICTED:
                        dist[conn.zone_a.name] += 1
                    queue.append(conn.zone_a)
        return dist

    def is_available_connection(self, connection: Connection) -> bool:
        """Check whether a connection edge and its target zone have capacity.

        Args:
            connection: The connection edge to evaluate.

        Returns:
            bool: True if both link capacity and target zone capacity are > 0.
        """
        if connection.zone_b.zone_type == ZoneType.RESTRICTED:
            return connection.max_link_capacity > 0
        return (
            connection.max_link_capacity > 0
            and connection.zone_b.max_drones > 0
        )

    def best_available_connection(
        self, connections: List[Connection]
    ) -> Optional[Connection]:
        """Select the best outgoing connection by zone type and distance.

        Preference order:
        1. PRIORITY zones with available capacity (sorted by distance to end)
        2. NORMAL zones with available capacity (sorted by distance to end)
        3. RESTRICTED zones with available capacity (sorted by distance to end)

        Args:
            connections: Candidate outgoing connections.

        Returns:
            Connection | None: Best connection choice or None if all are
                saturated or blocked.
        """
        # 1. Try PRIORITY connections first
        priority_connections = [
            connection
            for connection in connections
            if connection.zone_b.zone_type == ZoneType.PRIORITY
            and not connection.blocked
            and self.is_available_connection(connection)
        ]
        if priority_connections:
            return min(
                priority_connections,
                key=lambda c: self.dist_to_end.get(
                    c.zone_b.name, float("inf")
                ),
            )

        # 2. Try NORMAL connections second
        normal_connections = [
            connection
            for connection in connections
            if not connection.blocked
            and self.is_available_connection(connection)
        ]
        if normal_connections:
            return min(
                normal_connections,
                key=lambda c: self.dist_to_end.get(
                    c.zone_b.name, float("inf")
                ),
            )

        # 3. Try RESTRICTED connections last
        restricted_connections = [
            connection
            for connection in connections
            if connection.zone_b.zone_type == ZoneType.RESTRICTED
            and not connection.blocked
            and self.is_available_connection(connection)
        ]
        if restricted_connections:
            return min(
                restricted_connections,
                key=lambda c: self.dist_to_end.get(
                    c.zone_b.name, float("inf")
                ),
            )
        return None

    def neighboring_connections(
        self, region: Zone | Connection
    ) -> List[Connection]:
        """Retrieves outgoing connections attached to a Zone or Connection.

        Args:
            region: Current Zone or Connection object.

        Returns:
            list[Connection]: List of adjacent outgoing connection edges.

        Raises:
            ValueError: If region is not a Zone or Connection.
        """
        if isinstance(region, Zone):
            return self.graph.adjacency[region.name]
        elif isinstance(region, Connection):
            return self.graph.adjacency[region.zone_b.name]
        else:
            raise ValueError(
                "Invalid region type. Must be Zone or Connection."
            )

    def schedule_arriving_drones(
        self,
        arriving_drones_status: Deque[DroneStatus],
        current_connection: Connection | Zone,
        next_turn: Turn,
    ) -> None:
        """Schedules drones ready to move from their current location.

        Pick available next connections, move drones, update link/zone
        capacities, and record new reservations for ``next_turn``.

        Args:
            arriving_drones_status: Queue of DroneStatus objects ready for
                movement.
            current_connection: Zone or Connection currently occupied by the
                drones.
            next_turn: Accumulator Turn object for next turn reservations.
        """
        neighboring_connections = self.neighboring_connections(
            current_connection
        )
        while arriving_drones_status:
            next_connection = self.best_available_connection(
                neighboring_connections
            )
            if not next_connection:
                break
            status = arriving_drones_status.popleft()
            drone = status.drone
            drone.navigate(current_connection, next_connection)

            # Special case for end zone: end hub has unlimited capacity
            if (
                next_connection.zone_b == self.graph.end_zone
                and not drone.on_transit
            ):
                next_connection.zone_b.max_drones += 1

            next_turn.reservations.setdefault(next_connection.name, []).append(
                DroneStatus(
                    drone=drone, will_move=True, on_transit=drone.on_transit
                )
            )

        # Drones that could not be moved stay in their current position for
        # another turn.
        if arriving_drones_status:
            if (
                isinstance(current_connection, Connection)
                and current_connection.zone_b == self.graph.end_zone
            ):
                for status in arriving_drones_status:
                    if status.will_move:
                        current_connection.max_link_capacity += 1
            next_turn.reservations[current_connection.name] = [
                DroneStatus(drone=s.drone, will_move=False, on_transit=False)
                for s in arriving_drones_status
            ]

    def schedule_on_transit_drones(
        self,
        on_transit_drones_status: Deque[DroneStatus],
        current_connection: Connection | Zone,
        next_turn: Turn,
    ) -> None:
        """Schedule drones that are currently in multi-turn transit.

        Completes transit once destination zone capacity opens up.

        Args:
            on_transit_drones_status: Queue of DroneStatus objects in transit.
            current_connection: Connection edge currently being traversed.
            next_turn: Accumulator Turn object for next turn reservations.
        """
        if not isinstance(current_connection, Connection):
            return
        while (
            on_transit_drones_status
            and current_connection.zone_b.max_drones > 0
        ):
            status = on_transit_drones_status.popleft()
            drone = status.drone
            drone.navigate(current_connection, current_connection)
            if current_connection.zone_b == self.graph.end_zone:
                current_connection.zone_b.max_drones += 1
            next_turn.reservations.setdefault(
                current_connection.name, []
            ).append(
                DroneStatus(drone=drone, will_move=True, on_transit=False)
            )

        # If destination zone capacity is full, drones remain in transit
        if on_transit_drones_status:
            next_turn.reservations[current_connection.name] = [
                DroneStatus(
                    drone=s.drone,
                    will_move=False,
                    next_connection=current_connection.name,
                    on_transit=True,
                )
                for s in on_transit_drones_status
            ]

    def schedule_drones(self) -> None:
        """Executes the main turn-by-turn drone routing simulation loop.

        Prunes dead ends via DFS blocking, computes shortest paths to end_hub,
        and iterates through turns until all drones reach the destination hub.
        Finally calls `_generate_simulation_file()`.
        """
        start_zone = self.graph.start_zone
        # Block unusable dead-end branches
        self.graph.block(start_zone, [])
        if start_zone.zone_type == ZoneType.BLOCKED:
            return

        end_zone_name = self.graph.end_zone.name
        dist_to_end = self._compute_distances_to_end()
        self.dist_to_end = dist_to_end

        # Initial turn state: all drones at start_zone
        prev_turn_reservations: Dict[str, List[DroneStatus]] = {
            start_zone.name: [
                DroneStatus(drone=d, will_move=False, on_transit=False)
                for d in self.drones
            ]
        }

        # Simulation loop: continue until all reservations are at end_zone
        while any(
            end_zone_name not in connection_name
            for connection_name in prev_turn_reservations.keys()
        ):
            next_turn = Turn()
            prev_reservations = sorted(
                prev_turn_reservations.items(),
                key=lambda item: dist_to_end.get(
                    item[0].split("-")[1] if "-" in item[0] else item[0],
                    float("inf"),
                ),
            )

            # Process each active connection/zone reservation
            for connection_name, drones_status in prev_reservations:
                connection: Connection | Zone | None
                if "-" in connection_name:
                    zone_a_name, zone_b_name = connection_name.split("-")
                    connection = self.graph.connection(
                        zone_a_name, zone_b_name
                    )
                else:
                    connection = self.graph.zones[connection_name]

                if connection is None:
                    continue

                arriving_drones = deque(
                    status for status in drones_status if not status.on_transit
                )
                on_transit_drones = deque(
                    status for status in drones_status if status.on_transit
                )

                self.schedule_arriving_drones(
                    arriving_drones, connection, next_turn
                )
                self.schedule_on_transit_drones(
                    on_transit_drones, connection, next_turn
                )

            self.reservations.append(next_turn)
            prev_turn_reservations = next_turn.reservations

        # Clean up final capacities after simulation completes
        last_turn = self.reservations[-1]
        for connection_name, drones_status in last_turn.reservations.items():
            if "-" not in connection_name:
                continue
            zone_a, zone_b = connection_name.split("-")
            connection = self.graph.connection(zone_a, zone_b)
            if connection is None:
                continue
            for drone_status in drones_status:
                if drone_status.will_move:
                    connection.max_link_capacity += 1

        self._generate_simulation_file()

    def _generate_simulation_file(self) -> None:
        """Write turn-by-turn drone movement directives to the file."""
        with open("simulation_file.txt", "w") as f:
            for turn in self.reservations:
                line = ""
                for (
                    connection_name,
                    drones_status,
                ) in turn.reservations.items():
                    zone_name = (
                        connection_name.split("-")[1]
                        if "-" in connection_name
                        else connection_name
                    )
                    for drone_status in drones_status:
                        if drone_status.will_move:
                            if drone_status.on_transit:
                                line += (
                                    f"D{drone_status.drone.id}-"
                                    f"{connection_name} "
                                )
                            else:
                                line += (
                                    f"D{drone_status.drone.id}-{zone_name} "
                                )
                f.write(line + "\n")
