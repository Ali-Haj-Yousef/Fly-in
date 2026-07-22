"""
drone.py
========

Defines the Drone entity representing autonomous vehicles navigating
through the network of zones and connections.
"""

from connection import Connection
from zone import Zone, ZoneType


class Drone:
    """Represents a drone navigating through the network graph.

    Attributes:
        id: Unique numerical identifier for the drone.
        on_transit: Flag indicating whether the drone is currently spending
            an extra turn in transit (e.g. while entering a RESTRICTED zone).
    """

    def __init__(self, id: int):
        """Initializes a Drone instance with a given identifier.

        Args:
            id: Unique numerical identifier for the drone.
        """
        self.id = id
        self.on_transit = False

    def navigate(
            self, current_region: Connection | Zone,
            next_connection: Connection):
        """Navigates the drone from its current region along the target connection.

        Updates capacity counters for both the previous region (releasing capacity)
        and the target connection/destination zone (occupying capacity). If entering
        a RESTRICTED zone, sets `on_transit = True` for the extra delay turn.

        Args:
            current_region: The zone or connection where the drone is currently located.
            next_connection: The connection edge the drone intends to traverse.
        """
        # If remaining on the same connection (finishing transit inside restricted zone)
        if current_region == next_connection:
            next_connection.zone_b.max_drones -= 1
            self.on_transit = False
        else:
            # Release capacity from previous connection/zone if moving from a connection
            if isinstance(current_region, Connection):
                current_region.max_link_capacity += 1
                current_region.zone_b.max_drones += 1

            # Occupy capacity on the next connection
            next_connection.max_link_capacity -= 1

            # If target zone is RESTRICTED, mark drone as on transit (2-turn cost)
            if next_connection.zone_b.zone_type == ZoneType.RESTRICTED:
                self.on_transit = True
            else:
                # Decrement available capacity for normal target zone
                next_connection.zone_b.max_drones -= 1

