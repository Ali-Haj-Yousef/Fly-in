"""
fly_in.py
========

Main entry point for running the Fly-in drone network simulation
and visualization.

Usage:
    python fly_in.py <map_file>
"""

import sys

from parser import MapParser, ParserError
from scheduler import Scheduler
from drone import Drone
from visualizer import GUIVisualizer, TerminalVisualizer


def get_map_file_name() -> str:
    """Extract and validate the map file path from the CLI arguments.

    Returns:
        str: Path to the map file.

    Raises:
        ValueError: If the CLI arguments are invalid.
    """
    if len(sys.argv) != 2:
        raise ValueError("Usage: python3 fly_in.py <map_file>")
    return sys.argv[1]


def main() -> None:
    """Run the full Fly-in pipeline for parsing, scheduling, and display."""
    try:
        # 1. Parse command-line argument and map configuration file
        map_file_name = get_map_file_name()
        parser = MapParser()
        graph = parser.parse_file(map_file_name)

        # 2. Instantiate drone fleet and run scheduling algorithm
        scheduler = Scheduler(
            graph, [Drone(i) for i in range(graph.nb_drones)]
        )
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
