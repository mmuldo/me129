from __future__ import annotations
from typing import Dict, List, Optional, Tuple
import robot

class Intersection:
    def __init__(self, long: int, lat: int):
        self.long = long
        self.lat = lat

        # initialize streets to None (which means "unexplored")
        # True --> there's a street in that direction
        # False --> there's no street in that direction
        # Recall that the directions are [north, west, south, east]
        #self.streets = [None] * 4

    #def update_street(self, street: int, state: Optional[bool] = None):
    #    self.streets[street % 4] = state

    def __eq__(self, other):
        if isinstance(other, Intersection):
            return self.long == other.long and self.lat == other.lat
        return False

    def __hash__(self):
        return (self.long, self.lat).__hash__()

    def __add__(self, other):
        return Intersection(self.long + other.long, self.lat + other.lat)

    def __sub__(self, other):
        return Intersection(self.long - other.long, self.lat - other.lat)

    def __str__(self):
        return str((self.long, self.lat))

# Cardinal directions
N = 0
W = 1
S = 2
E = 3

dir_to_diff = [
    Intersection(0, 1),
    Intersection(-1, 0),
    Intersection(0, -1),
    Intersection(1, 0),
]

# Directions
F = 0   # forwards
L = 1   # left
B = 2   # backwards
R = 3   # right

diff_to_dir = {
    Intersection(0, 1):     F,
    Intersection(-1, 0):    L,
    Intersection(0, -1):    B,
    Intersection(1, 0):     R,
}

def route_to_directions(route: List[Intersection], heading: int):
    curr_heading = heading
    dirs = []
    for j in range(len(route)-1):
        direction = diff_to_dir[route[j+1]-route[j]]
        dirs.append(
            (
                direction - curr_heading
            ) % 4
        )
        curr_heading = direction

    return (dirs, curr_heading)




class Map:
    def __init__(self, intersections):
        self.intersections = intersections

    def add_street(
        self, 
        int1: Intersection,
        int2: Intersection
    ):

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
    def shortest_route(self, src: Intersection, dest: Intersection):
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
        return '\n'.join([
            f'{intersection}: {",".join([str(n) for n in neighbors])}' 
            for intersection, neighbors in self.intersections.items()
        ])


def build_map(bot: robot.EEBot, start: Intersection, heading: int):
    m = Map({})
    s = start
    curr_int = s

    # Mark all the vertices as not visited
    visited = []
 
    # Create a queue for BFS
    queue = []

    # Mark the source node as
    # visited and enqueue it
    queue.append(s)
    visited.append(s)
 
    while queue:
        print(m)
        print(curr_int)
        print(heading)
 
        # Dequeue a vertex from
        # queue and print it
        s = queue.pop(0)
        directions, heading = route_to_directions( 
            m.shortest_route(curr_int, s),
            heading
        )
        bot.follow_directions(directions)
        curr_int = s
 
        streets, heading = bot.scan(heading)

        for dir, street in enumerate(streets):
            if not street:
                continue

            new_int = curr_int + dir_to_diff[dir]
            if new_int not in visited:
                m.add_street(curr_int, new_int)
                queue.append(new_int)
                visited.append(new_int)
    return m
       

map1 = Map({})
map1.add_street(Intersection(0,0), Intersection(0,1))
map1.add_street(Intersection(0,0), Intersection(-1,0))
map1.add_street(Intersection(0,1), Intersection(0,2))
map1.add_street(Intersection(0,1), Intersection(-1,1))
map1.add_street(Intersection(-1,0), Intersection(-2,0))
map1.add_street(Intersection(0,2), Intersection(-1,2))
map1.add_street(Intersection(-1,1), Intersection(-1,2))
map1.add_street(Intersection(-1,1), Intersection(-2,1))
map1.add_street(Intersection(-2,0), Intersection(-2,1))
map1.add_street(Intersection(-2,0), Intersection(-3,0))
map1.add_street(Intersection(-2,1), Intersection(-3,1))
map1.add_street(Intersection(-3,0), Intersection(-3,1))

map2 = Map({})
map2.add_street(Intersection(0,0), Intersection(1,0))
map2.add_street(Intersection(0,0), Intersection(0,1))
map2.add_street(Intersection(1,0), Intersection(2,0))
map2.add_street(Intersection(1,0), Intersection(1,1))
map2.add_street(Intersection(2,0), Intersection(2,1))
map2.add_street(Intersection(1,1), Intersection(1,2))
map2.add_street(Intersection(2,1), Intersection(3,1))
map2.add_street(Intersection(2,1), Intersection(2,2))
map2.add_street(Intersection(1,2), Intersection(0,2))
map2.add_street(Intersection(1,2), Intersection(2,2))
map2.add_street(Intersection(3,1), Intersection(3,0))
map2.add_street(Intersection(3,1), Intersection(3,2))
map2.add_street(Intersection(2,2), Intersection(3,2))

map3 = Map({})
map3.add_street(Intersection(0,0), Intersection(1,0))
map3.add_street(Intersection(0,0), Intersection(0,1))
map3.add_street(Intersection(1,0), Intersection(2,0))
map3.add_street(Intersection(1,0), Intersection(1,1))
map3.add_street(Intersection(0,1), Intersection(1,1))
map3.add_street(Intersection(0,1), Intersection(0,2))
map3.add_street(Intersection(2,0), Intersection(3,0))
map3.add_street(Intersection(1,1), Intersection(2,1))
map3.add_street(Intersection(1,1), Intersection(1,2))
map3.add_street(Intersection(0,2), Intersection(1,2))
map3.add_street(Intersection(3,0), Intersection(3,1))
map3.add_street(Intersection(2,1), Intersection(3,1))
map3.add_street(Intersection(2,1), Intersection(2,2))
map3.add_street(Intersection(1,2), Intersection(2,2))
