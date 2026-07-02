# pathfinder.py
import heapq
from typing import List, Dict, Set, Tuple, Optional
from parser import Graph, ZoneType
from reservation import ReservationTable

class Pathfinder:
    def __init__(self, graph: Graph, reservations: Optional[ReservationTable] = None):
        self.graph = graph
        self.reservations = reservations

    def static_shortest_path(self, start_name: str, end_name: str) -> List[str]:
        start = self.graph.zones[start_name]
        end = self.graph.zones[end_name]
        pq = [(0, start_name, [start_name])]
        visited: Set[str] = set()
        dist: Dict[str, int] = {start_name: 0}
        while pq:
            cost, current, path = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)
            if current == end_name:
                return path
            zone = self.graph.zones[current]
            for conn in self.graph.neighbors(zone):
                neighbor = conn.other_end(zone)
                if neighbor.zone_type == ZoneType.BLOCKED:
                    continue
                new_cost = cost + neighbor.movement_cost
                if new_cost < dist.get(neighbor.name, float('inf')):
                    dist[neighbor.name] = new_cost
                    heapq.heappush(pq, (new_cost, neighbor.name, path + [neighbor.name]))
        raise ValueError(f"No path from {start_name} to {end_name}")

    def space_time_path(self, start_name: str, end_name: str, start_turn: int = 0) -> List[Tuple[str, int]]:
        if self.reservations is None:
            raise ValueError("ReservationTable required")
        start_zone = self.graph.zones[start_name]
        end_zone = self.graph.zones[end_name]

        pq = [(0, start_turn, start_name, [(start_name, start_turn)])]
        visited = set()
        dist = {(start_name, start_turn): 0}

        while pq:
            cost, turn, current_name, path = heapq.heappop(pq)
            state = (current_name, turn)
            if state in visited:
                continue
            visited.add(state)

            if current_name == end_name:
                return path   # first time we pop end, it's optimal

            current_zone = self.graph.zones[current_name]

            # Wait
            wait_turn = turn + 1
            if self.reservations.is_zone_free(current_name, wait_turn, current_zone.max_drones):
                new_cost = cost + 1
                if new_cost < dist.get((current_name, wait_turn), float('inf')):
                    dist[(current_name, wait_turn)] = new_cost
                    heapq.heappush(pq, (new_cost, wait_turn, current_name, path + [(current_name, wait_turn)]))

            # Move to neighbors
            for conn in self.graph.neighbors(current_zone):
                neighbor = conn.other_end(current_zone)
                if neighbor.zone_type == ZoneType.BLOCKED:
                    continue

                move_cost = neighbor.movement_cost
                arrival_turn = turn + move_cost

                # Check destination zone capacity
                if not self.reservations.is_zone_free(neighbor.name, arrival_turn, neighbor.max_drones):
                    continue

                # Check connection capacity for the whole interval
                if not self.reservations.is_connection_free_interval(
                    conn.name, turn, arrival_turn, conn.max_link_capacity
                ):
                    continue

                new_cost = cost + move_cost
                if new_cost < dist.get((neighbor.name, arrival_turn), float('inf')):
                    dist[(neighbor.name, arrival_turn)] = new_cost
                    heapq.heappush(pq, (new_cost, arrival_turn, neighbor.name, path + [(neighbor.name, arrival_turn)]))

        raise ValueError(f"No space-time path from {start_name} to {end_name}")
