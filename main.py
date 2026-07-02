# main.py
import sys
from parser import MapParser
from scheduler import Scheduler
from simulation import Simulation

def main(map_file):
    parser = MapParser()
    graph = parser.parse_file(map_file)
    scheduler = Scheduler(graph)
    drones = scheduler.schedule_drones(graph.nb_drones)
    sim = Simulation(graph, drones)
    sim.run()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python main.py <map_file>")
        sys.exit(1)
    main(sys.argv[1])
