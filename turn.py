from typing import Dict
from drone import Drone
from connection import Connection
from zone import Zone


class Turn:
    def __init__(self):
        self.drones_per_connections: Dict[str, list[Drone]] = {}
