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

    # Function to find the shortest
    # path between two nodes of a graph
    def shortest_route(self, src, dest):
        '''uses BFS since this is an unweighted graph'''
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
        for intersection, neighbors in self.intersections.items():
            print(f'{intersection}: {",".join([str(n) for n in neighbors])}')

# Driver Code
if __name__ == "__main__":
    
    A = Intersection(0,0)
    B = Intersection(1,1)
    C = Intersection(2,3)
    D = Intersection(5,-1)
    E = Intersection(6,4)
    F = Intersection(1,2)
    G = Intersection(10,9)

    # Graph using dictionaries
    graph = Map({A: [B, E, C],
            B: [A, D, E],
            C: [A, F, G],
            D: [B, E],
            E: [A, B, D],
            F: [C],
            G: [C]})
    
    # Function Call
    print(*graph.shortest_route(A, D))


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

m.add_street
