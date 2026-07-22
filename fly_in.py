"""
fly_in.py
========

Main entry point for running the Fly-in drone network simulation and visualization.

Usage:
    python fly_in.py <map_file>
"""

import sys

from parser import MapParser, ParserError
from scheduler import Scheduler
from drone import Drone
from turn import Turn
from visualizer import TerminalVisualizer, GUIVisualizer


def get_map_file_name() -> str:
    """Extracts and validates the map file path argument from command-line arguments.

    Returns:
        str: Path to the map file.

    Raises:
        ValueError: If command-line arguments are invalid.
    """
    if len(sys.argv) != 2:
        raise ValueError("Usage: python3 fly_in.py <map_file>")
    return sys.argv[1]


def main() -> None:
    """Runs the full Fly-in pipeline: parsing, scheduling, terminal rendering, and GUI visualization."""
    try:
        # 1. Parse command-line argument and map configuration file
        map_file_name = get_map_file_name()
        parser = MapParser()
        graph = parser.parse_file(map_file_name)

        # 2. Instantiate drone fleet and run scheduling algorithm
        scheduler = Scheduler(graph, [Drone(i) for i in range(graph.nb_drones)])
        scheduler.schedule_drones()

        # 3. Display terminal simulation output
        tv = TerminalVisualizer(graph, scheduler.reservations)
        tv.display(delay=0.3)

        # 4. Launch GUI visualizer
        gv = GUIVisualizer(graph, scheduler.reservations, speed_ms=800)
        gv.show()

    except (ParserError, FileNotFoundError) as e:
        print(e)


if __name__ == "__main__":
    main()

