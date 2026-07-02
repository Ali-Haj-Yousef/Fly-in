# simulation.py
from typing import List
from parser import Graph
from drone import Drone

class Simulation:
    def __init__(self, graph: Graph, drones: List[Drone]):
        self.graph = graph
        self.drones = sorted(drones, key=lambda d: d.drone_id)
        self.turn = 0
        self.delivered_count = 0

    def run(self):
        while self.delivered_count < len(self.drones):
            tokens = []
            for drone in self.drones:
                if not drone.delivered:
                    tok = drone.output_token()
                    if tok:
                        tokens.append(tok)
            if tokens:
                print(" ".join(tokens))
            else:
                print()  # blank line for turns with no moves (optional)

            for drone in self.drones:
                if not drone.delivered:
                    drone.advance()
                    if drone.delivered:
                        self.delivered_count += 1
            self.turn += 1
