from collections import deque
from connection import Connection
from drone import Drone
from graph import Graph
from zone import Zone, ZoneType
from turn import Turn


class Scheduler:
    def __init__(self, graph: Graph, drones: list[Drone]):
        self.graph = graph
        self.drones = drones

    def _compute_distances_to_end(self) -> dict:
        """Reverse BFS to compute hop distance from each zone to the end."""
        end = self.graph.end_zone
        dist = {end.name: 0}
        queue = deque([end])
        while queue:
            zone = queue.popleft()
            for conn in self.graph.connections:
                if conn.blocked:
                    continue
                if conn.zone_b.name == zone.name and conn.zone_a.name not in dist:
                    dist[conn.zone_a.name] = dist[zone.name] + 1
                    queue.append(conn.zone_a)
        return dist

    def is_available_connection(self, connection: Connection) -> bool:
        """
        Check if a connection is available for a drone to occupy.

        Args:
            connection: The connection to check.
        """
        if connection.zone_b.zone_type == ZoneType.RESTRICTED:
            return connection.max_link_capacity > 0
        return connection.max_link_capacity > 0 and connection.zone_b.max_drones > 0

    def best_available_connection(self, connections: list[Connection]) -> Connection | None:
        priority_connections = [
            connection for connection in connections
            if connection.zone_b.zone_type == ZoneType.PRIORITY and
            not connection.blocked and self.is_available_connection(connection)
        ]
        if priority_connections:
            return min(priority_connections, key=lambda c: self.dist_to_end.get(c.zone_b.name, float('inf')))
        fast_connections = [
            connection for connection in connections
            if connection.related_to_shortest_path and
            not connection.blocked and self.is_available_connection(connection)
        ]
        if fast_connections:
            return min(fast_connections, key=lambda c: self.dist_to_end.get(c.zone_b.name, float('inf')))
        normal_connections = [connection for connection in connections if not connection.blocked and self.is_available_connection(connection)]
        if normal_connections:
            return min(normal_connections, key=lambda c: self.dist_to_end.get(c.zone_b.name, float('inf')))
        restricted_connections = [connection for connection in connections if connection.zone_b.zone_type == ZoneType.RESTRICTED and not connection.blocked and self.is_available_connection(connection)]
        if restricted_connections:
            return min(restricted_connections, key=lambda c: self.dist_to_end.get(c.zone_b.name, float('inf')))
        return None

    def neighboring_connections(self, region: Zone | Connection) -> list[Connection]:
        """
        Get the neighboring connections of a given region.

        Args:
            region: The region to get the neighbors for.
        """
        if isinstance(region, Zone):
            return self.graph.adjacency[region.name]
        elif isinstance(region, Connection):
            return self.graph.adjacency[region.zone_b.name]
        else:
            raise ValueError("Invalid region type. Must be Zone or Connection.")

    def schedule_arriving_drones(self, arriving_drones: deque[Drone], current_connection: Connection, next_turn: Turn):
        neighboring_connections = self.neighboring_connections(current_connection)
        while arriving_drones:
            next_connection = self.best_available_connection(neighboring_connections)
            if not next_connection:
                break
            drone = arriving_drones.popleft()
            drone.navigate(current_connection, next_connection)
            if next_connection.zone_b == self.graph.end_zone:
                next_connection.zone_b.max_drones += 1
                next_connection.max_link_capacity += 1
            next_turn.drones_per_connections.setdefault(next_connection.name, []).append(drone)
        if arriving_drones:
            next_turn.drones_per_connections[current_connection.name] = list(arriving_drones)

    def schedule_on_transit_drones(self, on_transit_drones: deque[Drone], current_connection: Connection, next_turn: Turn):
        if not isinstance(current_connection, Connection):
            return
        while on_transit_drones and current_connection.zone_b.max_drones > 0:
            drone = on_transit_drones.popleft()
            drone.navigate(current_connection, current_connection)
            if current_connection.zone_b == self.graph.end_zone:
                current_connection.zone_b.max_drones += 1
                current_connection.max_link_capacity += 1
            next_turn.drones_per_connections.setdefault(current_connection.name, []).append(drone)
        if on_transit_drones:
            next_turn.drones_per_connections[current_connection.name] = list(on_transit_drones)

    def schedule(self):
        start_zone = self.graph.start_zone
        self.graph.block(start_zone, [])
        turns = []
        end_zone_name = self.graph.end_zone.name
        dist_to_end = self._compute_distances_to_end()
        self.dist_to_end = dist_to_end
        prev_turn_drones_per_connections = {start_zone.name: self.drones}
        while any(end_zone_name not in connection_name for connection_name in prev_turn_drones_per_connections.keys()):
            next_turn = Turn()
            sorted_items = sorted(
                prev_turn_drones_per_connections.items(),
                key=lambda item: dist_to_end.get(
                    item[0].split('-')[1] if '-' in item[0] else item[0],
                    float('inf')
                )
            )
            for connection_name, drones in sorted_items:
                if '-' in connection_name:
                    zone_a_name, zone_b_name = connection_name.split('-')
                    connection = self.graph.connection(zone_a_name, zone_b_name)
                else:
                    connection = self.graph.zones[connection_name]
                arriving_drones = deque(drone for drone in drones if not drone.on_transit)
                on_transit_drones = deque(drone for drone in drones if drone.on_transit)
                self.schedule_arriving_drones(arriving_drones, connection, next_turn)
                self.schedule_on_transit_drones(on_transit_drones, connection, next_turn)
            turns.append(next_turn)
            prev_turn_drones_per_connections = next_turn.drones_per_connections
        return turns
        # connections must be stored in the dict instead of zones (drones_per_zone)
