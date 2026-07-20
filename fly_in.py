import sys

from parser import MapParser, ParserError
from scheduler import Scheduler
from drone import Drone
from turn import Turn


def get_map_file_name() -> str:
    if len(sys.argv) != 2:
        raise ValueError("Usage: python3 fly_in.py <map_file>")
    return sys.argv[1]


def main():
    try:
        map_file_name = get_map_file_name()
        parser = MapParser()
        graph = parser.parse_file(map_file_name)
        # ad = graph.adjacency
        # for zone_name, connections in ad.items():
        #     print(f"{zone_name}: {[conn.name for conn in connections]}")
        # path = graph.shortest_path
        # for zone in path:
        #     print(zone.name)
        # conn = graph.connections
        # for con in conn:
        #     print(con, con.related_to_shortest_path)
        


        sc = Scheduler(graph, [Drone(i) for i in range(graph.nb_drones)])
        turns: list[Turn] = sc.schedule()
        print(f"turns nb = {len(turns)}")
        for i in range(len(turns)):
            print(f"Turn {i + 1}:")
            for connection_name, drones in turns[i].drones_per_connections.items():
                print(f"{connection_name} :")
                for drone in drones:
                    print(drone.id)
            print()
        print()

        # graph.block(graph.start_zone, [])
        # cons = [con for con in graph.connections if con.blocked]
        # for con in cons:
        #     print(con)

        # for zone in graph.shortest_path:
        #     print(zone)
    except (ParserError, FileNotFoundError) as e:
        print(e)


if __name__ == "__main__":
    main()
