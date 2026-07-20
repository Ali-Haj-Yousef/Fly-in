from connection import Connection
from zone import Zone, ZoneType


class Drone:
    def __init__(self, id):
        self.id = id
        self.on_transit = False

    def navigate(self, current_region: Connection | Zone, next_connection: Connection):
        if current_region == next_connection:
            next_connection.zone_b.max_drones -= 1
            self.on_transit = False
        else:
            if isinstance(current_region, Connection):
                current_region.max_link_capacity += 1
                current_region.zone_b.max_drones += 1
            next_connection.max_link_capacity -= 1
            if next_connection.zone_b.zone_type == ZoneType.RESTRICTED:
                self.on_transit = True
            else:
                next_connection.zone_b.max_drones -= 1
