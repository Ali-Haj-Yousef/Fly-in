# scheduler.py
from typing import List, Tuple
from parser import Graph
from reservation import ReservationTable
from pathfinder import Pathfinder
from drone import Drone, ScheduledStep, StepKind

class Scheduler:
    def __init__(self, graph: Graph):
        self.graph = graph
        self.reservations = ReservationTable()
        self.pathfinder = Pathfinder(graph, self.reservations)

    def schedule_drones(self, nb_drones: int) -> List[Drone]:
        start = self.graph.get_start_zone().name
        end = self.graph.get_end_zone().name

        # 1. Static paths for ordering heuristic
        static_paths = [self.pathfinder.static_shortest_path(start, end) for _ in range(nb_drones)]
        order = sorted(range(nb_drones), key=lambda i: len(static_paths[i]))

        # 2. Schedule each drone
        drones = []
        for idx in order:
            path = self.pathfinder.space_time_path(start, end, start_turn=0)
            schedule = self._build_schedule(path)
            self._commit_path(path)
            drone = Drone(drone_id=idx+1, schedule=schedule)
            drones.append(drone)

        # 3. Return sorted by ID
        return sorted(drones, key=lambda d: d.drone_id)

    def _build_schedule(self, path: List[Tuple[str, int]]) -> List[ScheduledStep]:
        # Build a schedule containing one ScheduledStep per turn (0..last_turn)
        if not path:
            return []

        last_turn = path[-1][1]
        schedule: List[Optional[ScheduledStep]] = [None] * (last_turn + 1)

        # Helper to set a step, overwriting previous if necessary
        def set_step(step: ScheduledStep) -> None:
            schedule[step.turn] = step

        # Initialize first entry
        first_zone, first_turn = path[0]
        if first_turn != 0:
            raise ValueError("Path must start at turn 0")
        set_step(ScheduledStep(turn=0, kind=StepKind.WAIT, zone=first_zone))

        # Process transitions
        for i in range(1, len(path)):
            prev_zone, prev_turn = path[i-1]
            zone, turn = path[i]
            diff = turn - prev_turn

            # Staying in same zone for one or more turns
            if prev_zone == zone:
                for t in range(prev_turn + 1, turn + 1):
                    set_step(ScheduledStep(turn=t, kind=StepKind.WAIT, zone=zone))
                continue

            # Movement between different zones
            conn = self.graph.get_connection(prev_zone, zone)
            if conn is None:
                raise ValueError(f"No connection between {prev_zone} and {zone}")

            if diff == 1:
                # Simple move: arrival at `turn` is a MOVE step
                set_step(ScheduledStep(turn=turn, kind=StepKind.MOVE, zone=zone))
            elif diff == 2:
                # Restricted move taking 2 turns: pattern ENTER_TRANSIT, WAIT, COMPLETE_TRANSIT
                set_step(ScheduledStep(turn=prev_turn, kind=StepKind.ENTER_TRANSIT, zone=prev_zone, connection=conn.name))
                set_step(ScheduledStep(turn=prev_turn + 1, kind=StepKind.WAIT, zone=prev_zone))
                set_step(ScheduledStep(turn=prev_turn + 2, kind=StepKind.COMPLETE_TRANSIT, zone=zone))
            else:
                raise ValueError(f"Unsupported move duration: {diff} turns between {prev_zone} and {zone}")

        # Fill any remaining None entries defensively with WAIT at previous zone
        last_known_zone = schedule[0].zone  # type: ignore[attr-defined]
        for t in range(len(schedule)):
            if schedule[t] is None:
                set_step(ScheduledStep(turn=t, kind=StepKind.WAIT, zone=last_known_zone))
            else:
                last_known_zone = schedule[t].zone

        # Convert to concrete list
        return list(schedule)

    def _commit_path(self, path: List[Tuple[str, int]]):
        # Book zones
        for zone, turn in path:
            self.reservations.book_zone(zone, turn)

        # Book connections for each transition
        for i in range(len(path) - 1):
            zone_a, turn_a = path[i]
            zone_b, turn_b = path[i+1]
            # If the drone stays in the same zone between these time steps
            # (a wait), there is no connection to reserve.
            if zone_a == zone_b:
                continue
            conn = self.graph.get_connection(zone_a, zone_b)
            if conn is None:
                raise ValueError(f"No connection between {zone_a} and {zone_b}")
            # Book for interval [turn_a, turn_b)
            self.reservations.book_connection_interval(conn.name, turn_a, turn_b)
