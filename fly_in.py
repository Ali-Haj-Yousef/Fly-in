import sys

from parser import MapParser, ParserError


def get_map_file_name() -> str:
    if len(sys.argv) != 2:
        raise ValueError("Usage: python3 fly_in.py <map_file>")
    return sys.argv[1]


def main():
    try:
        map_file_name = get_map_file_name()
        parser = MapParser()
        graph = parser.parse_file(map_file_name)

        print(graph)
        print(f"Start zone: {graph.get_start_zone()}")
        print(f"End zone:   {graph.get_end_zone()}")
        print("\nZones:")
        for zone in graph.zones.values():
            print(f"  {zone}")
        print("\nConnections:")
        for connection in graph.connections:
            print(f"  {connection}")
    except (ParserError, FileNotFoundError) as e:
        print(e)


if __name__ == "__main__":
    main()
