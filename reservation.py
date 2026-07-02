# reservation.py
from collections import defaultdict
from typing import Dict, Tuple

class ReservationTable:
    def __init__(self):
        self.zone_bookings: Dict[Tuple[str, int], int] = defaultdict(int)
        self.connection_bookings: Dict[Tuple[str, int], int] = defaultdict(int)

    def book_zone(self, zone_name: str, turn: int, count: int = 1):
        self.zone_bookings[(zone_name, turn)] += count

    def is_zone_free(self, zone_name: str, turn: int, capacity: int) -> bool:
        return self.zone_bookings.get((zone_name, turn), 0) < capacity

    def book_connection(self, conn_name: str, turn: int, count: int = 1):
        self.connection_bookings[(conn_name, turn)] += count

    def book_connection_interval(self, conn_name: str, start_turn: int, end_turn: int, count: int = 1):
        for t in range(start_turn, end_turn):
            self.book_connection(conn_name, t, count)

    def is_connection_free_at(self, conn_name: str, turn: int, capacity: int) -> bool:
        return self.connection_bookings.get((conn_name, turn), 0) < capacity

    def is_connection_free_interval(self, conn_name: str, start_turn: int, end_turn: int, capacity: int) -> bool:
        for t in range(start_turn, end_turn):
            if not self.is_connection_free_at(conn_name, t, capacity):
                return False
        return True
