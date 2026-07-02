"""
drone.py
========

Drone agent for the Fly-in multi-agent pathfinding simulation.

A Drone is a stateful agent that follows a pre-computed schedule through the
zone network.  The schedule is a sequence of :class:`ScheduledStep` objects
produced by the Scheduler/Pathfinder and assigned via :meth:`Drone.assign_schedule`.

Movement mechanics recap (from the subject)
-------------------------------------------
* **Normal / Priority zone** – 1-turn move.  The drone departs and arrives
  within the same simulation turn.
* **Restricted zone** – 2-turn move split into two consecutive steps:

    Turn T   → ``ENTER_TRANSIT``: drone leaves its zone and occupies the
                connection.  Output token: ``D<ID>-<connection_name>``.
    Turn T+1 → ``COMPLETE_TRANSIT``: drone must arrive at the restricted
                zone.  It **cannot** wait on the connection between the two
                steps.  Output token: ``D<ID>-<zone_name>``.

* **Wait** – drone stays in its current zone; omitted from the output line.

State machine
-------------
::

    IDLE ──► AT_ZONE ──► IN_TRANSIT ──► AT_ZONE (or ARRIVED)
                │                            ▲
                └────────────────────────────┘  (normal / priority move)
                │
                └──► ARRIVED  (move lands on end zone)

All states except ARRIVED allow a WAIT step (self-loop on AT_ZONE).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

from parser import Connection, Zone


# --------------------------------------------------------------------------- #
# Exceptions
# --------------------------------------------------------------------------- #


class DroneError(Exception):
    """Raised when a drone is asked to perform an illegal operation.

    Attributes:
        drone_label: The label of the offending drone (e.g. ``"D3"``).
        message: Human-readable description of the violation.
    """

    def __init__(self, drone_label: str, message: str) -> None:
        self.drone_label = drone_label
        self.message = message
        super().__init__(f"[{drone_label}] {message}")


# --------------------------------------------------------------------------- #
# Step-level enums and data
# --------------------------------------------------------------------------- #


class StepKind(Enum):
    """Describes what a drone does during a single simulation turn.

    Members
    -------
    WAIT
        The drone stays in its current zone.  Omitted from output.
    MOVE
        The drone traverses a connection and arrives at a normal or priority
        zone within this single turn.
    ENTER_TRANSIT
        First turn of a 2-turn restricted-zone move.  The drone leaves its
        current zone and occupies the connection.  Output shows the
        connection name.
    COMPLETE_TRANSIT
        Second (mandatory) turn of a 2-turn restricted-zone move.  The drone
        arrives at the restricted zone.  Output shows the zone name.
    """

    WAIT = "wait"
    MOVE = "move"
    ENTER_TRANSIT = "enter_transit"
    COMPLETE_TRANSIT = "complete_transit"


@dataclass(frozen=True)
class ScheduledStep:
    """Immutable record of a single planned action for a drone.

    One ``ScheduledStep`` exists for **every** simulation turn from the
    drone's departure until its arrival, including wait turns.  The
    Scheduler creates these; the Drone only reads them.

    Attributes:
        turn:        The simulation turn (0-indexed) this step applies to.
        kind:        The category of action (see :class:`StepKind`).
        destination: The zone the drone will occupy after this step.
                     Required for ``MOVE`` and ``COMPLETE_TRANSIT``;
                     ``None`` for ``WAIT`` and ``ENTER_TRANSIT``.
        connection:  The connection being traversed.
                     Required for ``MOVE``, ``ENTER_TRANSIT``, and
                     ``COMPLETE_TRANSIT``; ``None`` for ``WAIT``.

    Examples:
        >>> # Normal 1-turn move: hub → roof1
        >>> ScheduledStep(turn=1, kind=StepKind.MOVE,
        ...               destination=roof1_zone, connection=hub_roof1_conn)

        >>> # Restricted 2-turn move: zoneA → restricted_zone
        >>> ScheduledStep(turn=2, kind=StepKind.ENTER_TRANSIT,
        ...               destination=None, connection=a_restricted_conn)
        >>> ScheduledStep(turn=3, kind=StepKind.COMPLETE_TRANSIT,
        ...               destination=restricted_zone, connection=a_restricted_conn)
    """

    turn: int
    kind: StepKind
    destination: Optional[Zone] = None
    connection: Optional[Connection] = None

    def __post_init__(self) -> None:
        """Validates that the step is internally consistent."""
        if self.kind is StepKind.MOVE and self.destination is None:
            raise ValueError("MOVE step requires a destination zone")
        if self.kind is StepKind.MOVE and self.connection is None:
            raise ValueError("MOVE step requires a connection")
        if self.kind is StepKind.ENTER_TRANSIT and self.connection is None:
            raise ValueError("ENTER_TRANSIT step requires a connection")
        if self.kind is StepKind.COMPLETE_TRANSIT and self.destination is None:
            raise ValueError("COMPLETE_TRANSIT step requires a destination zone")
        if self.kind is StepKind.COMPLETE_TRANSIT and self.connection is None:
            raise ValueError("COMPLETE_TRANSIT step requires a connection")


# --------------------------------------------------------------------------- #
# Drone state enum
# --------------------------------------------------------------------------- #


class DroneState(Enum):
    """Represents the current activity state of a drone.

    Transitions
    -----------
    ``IDLE`` → ``AT_ZONE`` (once a schedule is assigned and first turn runs)
    ``AT_ZONE`` → ``AT_ZONE`` (WAIT step, self-loop)
    ``AT_ZONE`` → ``IN_TRANSIT`` (ENTER_TRANSIT step)
    ``AT_ZONE`` → ``ARRIVED`` (MOVE step to end zone)
    ``IN_TRANSIT`` → ``AT_ZONE`` (COMPLETE_TRANSIT step to non-end zone)
    ``IN_TRANSIT`` → ``ARRIVED`` (COMPLETE_TRANSIT step to end zone)
    ``ARRIVED`` → *(terminal – no further transitions)*
    """

    IDLE = "idle"
    AT_ZONE = "at_zone"
    IN_TRANSIT = "in_transit"
    ARRIVED = "arrived"


# --------------------------------------------------------------------------- #
# Drone
# --------------------------------------------------------------------------- #


class Drone:
    """A single drone agent navigating the zone network.

    The drone itself is *passive*: it holds a schedule and exposes methods
    that the :class:`Simulation` engine calls once per turn.  All routing
    decisions are made externally by the Scheduler.

    Attributes (read-only via properties):
        drone_id:      Integer identifier (1-indexed).
        label:         Human-readable label used in output (e.g. ``"D3"``).
        current_zone:  The zone the drone currently occupies.
        state:         Current :class:`DroneState`.
        is_arrived:    Convenience shortcut for ``state == ARRIVED``.
        is_active:     True when the drone still has moves to execute.
        arrival_turn:  The turn the drone reached the end zone (or ``None``).
        turns_waited:  Total number of WAIT steps in the schedule.
        path_cost:     Total weighted cost of the drone's path
                       (sum of zone movement costs, not raw turns).

    Example usage (from the Simulation engine)::

        drone.assign_schedule(steps)
        for turn in range(max_turns):
            token = drone.output_token(turn)   # read before advance
            if token:
                print(token)
            drone.advance(turn)
            if drone.is_arrived:
                break
    """

    def __init__(self, drone_id: int, start_zone: Zone) -> None:
        """Initialises the drone at the start zone before scheduling.

        Args:
            drone_id:   Positive integer identifier.  ``label`` will be
                        ``f"D{drone_id}"``.
            start_zone: The zone all drones share at simulation start.
        """
        if drone_id <= 0:
            raise DroneError(f"D{drone_id}", "drone_id must be a positive integer")

        self._id: int = drone_id
        self._label: str = f"D{drone_id}"
        self._current_zone: Zone = start_zone
        self._state: DroneState = DroneState.IDLE

        # Schedule: populated by assign_schedule(); Dict for O(1) turn lookup.
        self._steps: Dict[int, ScheduledStep] = {}

        # Transit state: tracked while IN_TRANSIT so the drone knows where
        # it is heading and which connection it is occupying.
        self._transit_destination: Optional[Zone] = None
        self._transit_connection: Optional[Connection] = None

        # Statistics (used by scoring / README metrics).
        self._arrival_turn: Optional[int] = None
        self._turns_waited: int = 0
        self._path_cost: int = 0

    # ------------------------------------------------------------------ #
    # Read-only properties
    # ------------------------------------------------------------------ #

    @property
    def drone_id(self) -> int:
        """Positive integer identifier of this drone."""
        return self._id

    @property
    def label(self) -> str:
        """Output label, e.g. ``"D1"``."""
        return self._label

    @property
    def current_zone(self) -> Zone:
        """The zone the drone currently occupies.

        During ``IN_TRANSIT`` this is the zone the drone *departed from*,
        since it has not yet arrived at its destination.
        """
        return self._current_zone

    @property
    def state(self) -> DroneState:
        """Current :class:`DroneState` of this drone."""
        return self._state

    @property
    def is_arrived(self) -> bool:
        """True once the drone has reached the end zone."""
        return self._state is DroneState.ARRIVED

    @property
    def is_active(self) -> bool:
        """True while the drone still has actions to execute."""
        return self._state is not DroneState.ARRIVED

    @property
    def arrival_turn(self) -> Optional[int]:
        """The simulation turn on which this drone reached the end zone.

        Returns ``None`` if the drone has not yet arrived.
        """
        return self._arrival_turn

    @property
    def turns_waited(self) -> int:
        """Total number of turns the drone spent waiting in place."""
        return self._turns_waited

    @property
    def path_cost(self) -> int:
        """Weighted movement cost of the full path (sum of zone entry costs).

        This is the cost metric used by the pathfinder, not the raw number
        of simulation turns.
        """
        return self._path_cost

    @property
    def transit_connection(self) -> Optional[Connection]:
        """The connection currently being traversed, or ``None``.

        Non-``None`` only while ``state == IN_TRANSIT``.
        """
        return self._transit_connection

    @property
    def transit_destination(self) -> Optional[Zone]:
        """The restricted zone the drone is flying toward, or ``None``.

        Non-``None`` only while ``state == IN_TRANSIT``.
        """
        return self._transit_destination

    @property
    def schedule(self) -> List[ScheduledStep]:
        """Returns the full schedule as a turn-ordered list (read-only copy)."""
        return sorted(self._steps.values(), key=lambda s: s.turn)

    # ------------------------------------------------------------------ #
    # Schedule assignment (called by Scheduler)
    # ------------------------------------------------------------------ #

    def assign_schedule(self, steps: List[ScheduledStep]) -> None:
        """Assigns a pre-computed schedule to this drone.

        Must be called before the first :meth:`advance` call.  May be
        called again to re-plan, which resets all mutable state.

        Args:
            steps: Ordered list of :class:`ScheduledStep` objects covering
                   every simulation turn from departure to arrival.

        Raises:
            DroneError: If the schedule is empty, contains duplicate turns,
                        or has an ENTER_TRANSIT not immediately followed by
                        COMPLETE_TRANSIT.
        """
        if not steps:
            raise DroneError(self._label, "Cannot assign an empty schedule")

        step_dict: Dict[int, ScheduledStep] = {}
        for step in steps:
            if step.turn in step_dict:
                raise DroneError(
                    self._label,
                    f"Duplicate step for turn {step.turn} in schedule",
                )
            step_dict[step.turn] = step

        self._validate_transit_pairs(steps)

        # Reset mutable state so the drone can be re-scheduled cleanly.
        self._steps = step_dict
        self._state = DroneState.AT_ZONE
        self._transit_destination = None
        self._transit_connection = None
        self._arrival_turn = None
        self._turns_waited = 0
        self._path_cost = self._compute_path_cost(steps)

    # ------------------------------------------------------------------ #
    # Per-turn interface (called by Simulation engine)
    # ------------------------------------------------------------------ #

    def output_token(self, turn: int) -> Optional[str]:
        """Returns the output string for the given turn, or ``None``.

        The simulation engine calls this **before** :meth:`advance` so that
        the token reflects the drone's action *during* the turn, not after.

        Output format:
            * ``"D<ID>-<zone_name>"``  – for MOVE and COMPLETE_TRANSIT.
            * ``"D<ID>-<connection_name>"`` – for ENTER_TRANSIT (in flight).
            * ``None`` – for WAIT steps or if the drone is already arrived.

        Args:
            turn: The current simulation turn (0-indexed).

        Returns:
            Optional[str]: Token string, or ``None`` if the drone produces
            no output this turn.
        """
        if self._state is DroneState.ARRIVED:
            return None

        step = self._steps.get(turn)
        if step is None or step.kind is StepKind.WAIT:
            return None

        if step.kind is StepKind.MOVE:
            # destination is guaranteed non-None by ScheduledStep validation
            dest = step.destination
            assert dest is not None  # narrow type for mypy
            return f"{self._label}-{dest.name}"

        if step.kind is StepKind.ENTER_TRANSIT:
            # connection is guaranteed non-None by ScheduledStep validation
            conn = step.connection
            assert conn is not None  # narrow type for mypy
            return f"{self._label}-{conn.name}"

        if step.kind is StepKind.COMPLETE_TRANSIT:
            dest = step.destination
            assert dest is not None  # narrow type for mypy
            return f"{self._label}-{dest.name}"

        return None  # unreachable, satisfies mypy

    def advance(self, turn: int) -> None:
        """Executes the scheduled action for the given turn, updating state.

        The simulation engine calls this **after** :meth:`output_token` so
        that all drones move simultaneously (tokens are collected first,
        then state is updated for all drones).

        Args:
            turn: The current simulation turn (0-indexed).

        Raises:
            DroneError: If a COMPLETE_TRANSIT step is missing after an
                        ENTER_TRANSIT (would leave the drone stranded on a
                        connection, which the subject forbids).
            DroneError: If advance is called on an already-arrived drone.
        """
        if self._state is DroneState.ARRIVED:
            raise DroneError(
                self._label,
                f"advance() called on turn {turn} but drone already arrived",
            )

        step = self._steps.get(turn)

        # No step scheduled for this turn: drone idles at its current zone.
        if step is None:
            return

        if step.kind is StepKind.WAIT:
            self._handle_wait()

        elif step.kind is StepKind.MOVE:
            self._handle_move(step, turn)

        elif step.kind is StepKind.ENTER_TRANSIT:
            self._handle_enter_transit(step, turn)

        elif step.kind is StepKind.COMPLETE_TRANSIT:
            self._handle_complete_transit(step, turn)

    # ------------------------------------------------------------------ #
    # State-transition handlers (private)
    # ------------------------------------------------------------------ #

    def _handle_wait(self) -> None:
        """Handles a WAIT step: drone stays in current zone."""
        # State remains AT_ZONE.  Being IN_TRANSIT and then waiting is
        # illegal (caught at schedule validation time), so no guard needed.
        self._state = DroneState.AT_ZONE
        self._turns_waited += 1

    def _handle_move(self, step: ScheduledStep, turn: int) -> None:
        """Handles a MOVE step: drone arrives at a normal/priority zone.

        Args:
            step: The MOVE step to execute.
            turn: The current simulation turn (for arrival_turn recording).
        """
        assert step.destination is not None  # validated at schedule assignment
        self._current_zone = step.destination

        if self._current_zone.is_end:
            self._state = DroneState.ARRIVED
            self._arrival_turn = turn
        else:
            self._state = DroneState.AT_ZONE

    def _handle_enter_transit(self, step: ScheduledStep, turn: int) -> None:
        """Handles an ENTER_TRANSIT step: drone occupies a connection for 1 turn.

        The current zone is NOT updated here; it will be updated by the
        subsequent COMPLETE_TRANSIT step.

        Args:
            step: The ENTER_TRANSIT step to execute.
            turn: Current simulation turn (for error context only).
        """
        assert step.connection is not None  # validated at schedule assignment

        # Guard: transition is only legal from AT_ZONE.
        if self._state is DroneState.IN_TRANSIT:
            raise DroneError(
                self._label,
                f"Turn {turn}: cannot ENTER_TRANSIT while already IN_TRANSIT",
            )

        self._state = DroneState.IN_TRANSIT
        self._transit_connection = step.connection

        # Infer the destination from the connection and the drone's current
        # zone (the destination is the *other* end of the connection).
        self._transit_destination = step.connection.other_end(self._current_zone)

    def _handle_complete_transit(self, step: ScheduledStep, turn: int) -> None:
        """Handles a COMPLETE_TRANSIT step: drone arrives at the restricted zone.

        Args:
            step: The COMPLETE_TRANSIT step to execute.
            turn: The current simulation turn (for arrival_turn recording).

        Raises:
            DroneError: If called while the drone is not IN_TRANSIT.
        """
        if self._state is not DroneState.IN_TRANSIT:
            raise DroneError(
                self._label,
                f"Turn {turn}: COMPLETE_TRANSIT requires IN_TRANSIT state, "
                f"but state is {self._state.value}",
            )

        assert step.destination is not None  # validated at schedule assignment
        self._current_zone = step.destination
        self._transit_connection = None
        self._transit_destination = None

        if self._current_zone.is_end:
            self._state = DroneState.ARRIVED
            self._arrival_turn = turn
        else:
            self._state = DroneState.AT_ZONE

    # ------------------------------------------------------------------ #
    # Schedule validation helpers (private)
    # ------------------------------------------------------------------ #

    def _validate_transit_pairs(self, steps: List[ScheduledStep]) -> None:
        """Ensures every ENTER_TRANSIT is immediately followed by COMPLETE_TRANSIT.

        The subject states: "the drone MUST reach its destination during the
        next turn. It can't wait extra turns on the connection."

        Args:
            steps: Schedule steps in arbitrary order.

        Raises:
            DroneError: If a transit pair is missing or mismatched.
        """
        sorted_steps = sorted(steps, key=lambda s: s.turn)
        for index, step in enumerate(sorted_steps):
            if step.kind is not StepKind.ENTER_TRANSIT:
                continue
            # The very next step must be COMPLETE_TRANSIT on the next turn.
            if index + 1 >= len(sorted_steps):
                raise DroneError(
                    self._label,
                    f"Turn {step.turn}: ENTER_TRANSIT has no following step",
                )
            next_step = sorted_steps[index + 1]
            if next_step.kind is not StepKind.COMPLETE_TRANSIT:
                raise DroneError(
                    self._label,
                    f"Turn {step.turn}: ENTER_TRANSIT must be immediately "
                    f"followed by COMPLETE_TRANSIT, got {next_step.kind.value}",
                )
            if next_step.turn != step.turn + 1:
                raise DroneError(
                    self._label,
                    f"Turn {step.turn}: ENTER_TRANSIT at turn {step.turn} "
                    f"must be followed by COMPLETE_TRANSIT at turn "
                    f"{step.turn + 1}, got turn {next_step.turn}",
                )

    @staticmethod
    def _compute_path_cost(steps: List[ScheduledStep]) -> int:
        """Computes the total weighted movement cost of the schedule.

        Only MOVE and COMPLETE_TRANSIT steps contribute (each destination
        zone's movement cost is added).  WAIT and ENTER_TRANSIT steps do
        not add path cost directly (the 2-turn restricted cost is already
        captured by the COMPLETE_TRANSIT destination's movement_cost=2).

        Args:
            steps: The full ordered schedule.

        Returns:
            int: Sum of destination zone movement costs across the path.
        """
        total = 0
        for step in steps:
            if step.kind in (StepKind.MOVE, StepKind.COMPLETE_TRANSIT):
                if step.destination is not None:
                    total += step.destination.movement_cost
        return total

    # ------------------------------------------------------------------ #
    # Dunder helpers
    # ------------------------------------------------------------------ #

    def __repr__(self) -> str:
        """Returns a concise developer-friendly string for this drone."""
        return (
            f"Drone(id={self._id}, zone={self._current_zone.name!r}, "
            f"state={self._state.value}, arrived={self.is_arrived})"
        )

    def __str__(self) -> str:
        """Returns the drone's label (e.g. ``'D3'``)."""
        return self._label


# --------------------------------------------------------------------------- #
# Factory helper
# --------------------------------------------------------------------------- #


def create_drones(nb_drones: int, start_zone: Zone) -> List[Drone]:
    """Creates and returns a fleet of ``nb_drones`` drones at ``start_zone``.

    Drone IDs are assigned starting from 1, so labels are D1, D2, …, Dn.

    Args:
        nb_drones:  Number of drones to create (must be positive).
        start_zone: The shared starting zone (from the parsed Graph).

    Returns:
        List[Drone]: Fleet of drones, ordered by ID ascending.

    Raises:
        ValueError: If ``nb_drones`` is not a positive integer.

    Example:
        >>> graph = MapParser().parse_file("map.txt")
        >>> fleet = create_drones(graph.nb_drones, graph.get_start_zone())
    """
    if nb_drones <= 0:
        raise ValueError(f"nb_drones must be positive, got {nb_drones}")
    return [Drone(drone_id=i, start_zone=start_zone) for i in range(1, nb_drones + 1)]


# --------------------------------------------------------------------------- #
# Smoke test
# --------------------------------------------------------------------------- #

if __name__ == "__main__":
    from parser import MapParser

    sample_map = """\
nb_drones: 2
start_hub: hub 0 0 [color=green]
end_hub: goal 10 10 [color=yellow]
hub: roof1 3 4 [zone=restricted color=red]
hub: corridorA 4 3 [zone=priority color=green]
connection: hub-roof1
connection: hub-corridorA
connection: roof1-goal
connection: corridorA-goal
"""
    graph = MapParser().parse_lines(sample_map.splitlines(keepends=True))
    fleet = create_drones(graph.nb_drones, graph.get_start_zone())

    hub = graph.zones["hub"]
    roof1 = graph.zones["roof1"]
    corridor = graph.zones["corridorA"]
    goal = graph.zones["goal"]
    conn_hub_roof1 = graph.connections[0]      # hub-roof1
    conn_hub_corridor = graph.connections[1]   # hub-corridorA
    conn_roof1_goal = graph.connections[2]     # roof1-goal
    conn_corridor_goal = graph.connections[3]  # corridorA-goal

    # D1: hub → roof1 (restricted, 2-turn) → goal (normal, 1-turn)
    d1_schedule = [
        ScheduledStep(turn=0, kind=StepKind.ENTER_TRANSIT,
                      connection=conn_hub_roof1),
        ScheduledStep(turn=1, kind=StepKind.COMPLETE_TRANSIT,
                      destination=roof1, connection=conn_hub_roof1),
        ScheduledStep(turn=2, kind=StepKind.MOVE,
                      destination=goal, connection=conn_roof1_goal),
    ]

    # D2: hub → corridorA (priority, 1-turn) → goal (normal, 1-turn)
    d2_schedule = [
        ScheduledStep(turn=0, kind=StepKind.MOVE,
                      destination=corridor, connection=conn_hub_corridor),
        ScheduledStep(turn=1, kind=StepKind.MOVE,
                      destination=goal, connection=conn_corridor_goal),
    ]

    fleet[0].assign_schedule(d1_schedule)
    fleet[1].assign_schedule(d2_schedule)

    print("=== Simulation output ===")
    for turn in range(5):
        tokens = [
            drone.output_token(turn)
            for drone in fleet
            if not drone.is_arrived
        ]
        active_tokens = [t for t in tokens if t is not None]
        if active_tokens:
            print(" ".join(active_tokens))
        for drone in fleet:
            if not drone.is_arrived:
                drone.advance(turn)
        if all(drone.is_arrived for drone in fleet):
            break

    print("\n=== Final stats ===")
    for drone in fleet:
        print(
            f"{drone.label}: arrived_turn={drone.arrival_turn}, "
            f"waited={drone.turns_waited}, path_cost={drone.path_cost}"
        )
