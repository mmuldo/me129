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

    def __sub__(self, other):
        return (self.long - other.long, self.lat - other.lat)

    def __str__(self):
        return str((self.long, self.lat))


class Map:
    def __init__(self, intersections):
        self.intersections = intersections
        self.long_lats = [(inter.long, inter.lat) for inter in intersections]

    def long_lat_to_intersection(
        self, 
        long_lat: Tuple[int, int]
    ) -> Intersection:
        if long_lat not in self.long_lats:
            self.long_lats.append(long_lat)
            return Intersection(*long_lat)
        else:
            return [
                intersection 
                for intersection in self.intersections 
                if intersection.long == long_lat[0] 
                and intersection.lat == long_lat[1]
            ][0]

    def add_street(
        self, 
        long_lat1: Tuple[int, int], 
        long_lat2: Tuple[int, int]
    ):

        int1 = self.long_lat_to_intersection(long_lat1)
        int2 = self.long_lat_to_intersection(long_lat2)

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


    # Function to find the shortest
    # path between two nodes of a graph
    def shortest_route(self, start, end):
        '''uses BFS since this is an unweighted graph'''

        src = self.long_lat_to_intersection(start)
        dest = self.long_lat_to_intersection(end)

        explored = []
        
        # Queue for traversing the
        # map in the BFS
        queue = [[src]]
        
        # If the desired intersection is
        # reached
        if src == dest:
            return [src]
        
        # Loop to traverse the map
        # with the help of the queue
        while queue:
            route = queue.pop(0)
            intersection = route[-1]
            
            # Condition to check if the
            # current intersection is not visited
            if intersection not in explored:
                neighbours = self.intersections[intersection]
                
                # Loop to iterate over the
                # neighbours of the node
                for neighbour in neighbours:
                    new_route = list(route)
                    new_route.append(neighbour)
                    queue.append(new_route)
                    
                    # Condition to check if the
                    # neighbour intersection is the destination
                    if neighbour == dest:
                        return new_route
                explored.append(intersection)
    
        # Condition when the intersections
        # are not connected
        return []

    def __str__(self):
        return '\n'.join([
            f'{intersection}: {",".join([str(n) for n in neighbors])}' 
            for intersection, neighbors in self.intersections.items()
        ])


def build_map(start: Intersection):
    pass
    

map1 = Map({})
map1.add_street((0,0), (0,1))
map1.add_street((0,0), (-1,0))
map1.add_street((0,1), (0,2))
map1.add_street((0,1), (-1,1))
map1.add_street((-1,0), (-2,0))
map1.add_street((0,2), (-1,2))
map1.add_street((-1,1), (-1,2))
map1.add_street((-1,1), (-2,1))
map1.add_street((-2,0), (-2,1))
map1.add_street((-2,0), (-3,0))
map1.add_street((-2,1), (-3,1))
map1.add_street((-3,0), (-3,1))
