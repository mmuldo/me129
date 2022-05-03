from __future__ import annotations
from typing import Dict, List, Optional, Tuple

class Intersection:
    def __init__(self, long: int, lat: int):
        self.long = long
        self.lat = lat

        # initialize streets to None (which means "unexplored")
        # True --> there's a street in that direction
        # False --> there's no street in that direction
        # Recall that the directions are [north, west, south, east]
        self.streets = [None] * 4

    def update_street(self, street: int, state: Optional[bool] = None):
        self.streets[street % 4] = state

    def __str__(self):
        return str((self.long, self.lat))


class Map:
    def __init__(self, intersections):
        self.intersections = intersections

    def add_street(self, int1, int2):
        # connect first intersection to second
        try:
            self.intersections[int1].append(int2)
        except KeyError:
            self.intersections[int1] = [int2]

        # connect second intersection to first
        try:
            self.intersections[int2].append(int1)
        except KeyError:
            self.intersections[int2] = [int1]

    def __str__(self):
        for intersection, neighbors in self.intersections.items():
            print(f'{intersection}: {",".join([str(n) for n in neighbors])}')


def build_map(start: Intersection):
    pass
    

int1 = Intersection(1, 2)
int2 = Intersection(3, 4)
int3 = Intersection(0, 6)
int4 = Intersection(7, 6)

m = Map({})
m.add_street(int1, int2)
m.add_street(int1, int4)
m.add_street(int2, int3)
m.print_intersections()

m.add_street
