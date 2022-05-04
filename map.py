from __future__ import annotations
from typing import Dict, List, Optional, Tuple
import robot

class Intersection:
    '''
    Represents an intersection in a map
    basically just a tuple (longitude, latitude)

    Attributes
    ----------
    long : int
        longitude (x coordinate) in map
    latitude : int
        latitude (y coordinate) in map
    '''
    def __init__(self, long: int, lat: int):
        self.long = long
        self.lat = lat

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
N = 0 #north
W = 1 #west
S = 2 #south
E = 3 #east

# converts cardinal direction to coordinate vector
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

# converts a coordinate vector to a direction
diff_to_dir = {
    Intersection(0, 1):     F,
    Intersection(-1, 0):    L,
    Intersection(0, -1):    B,
    Intersection(1, 0):     R,
}

def route_to_directions(
    route: List[Intersection], 
    heading: int
) -> Tuple[List[int], int]:
    '''
    converts a series of Intersections (i.e. coordinate points) to directions
    to follow

    Parameters
    ----------
    route : List[Intersection]
        sequence of Intersections in map to get from one intersection to another
    heading : int
        current heading

    Returns
    -------
    Tuple[List[int], int]
        first item is the sequence of directions (F, L, B, R) to follow
        second item is the new heading after completing directions
    '''
    curr_heading = heading
    dirs = []
    for j in range(len(route)-1):
        direction = diff_to_dir[route[j+1]-route[j]]
        dirs.append(
            ( direction - curr_heading) % 4
        )
        curr_heading = direction

    return (dirs, curr_heading)




class Map:
    '''
    represents map (graph in which the nodes are intersections and edges are
    streets)

    Attributes
    ----------
    intersections : Dict[Intersection, List[Intersection]]
        adjacency dictionary for nodes in graph (indicates the neighbors of
        each intersection)

    Methods
    -------
    add_street(int1, int2): adds street (edge) to map
    shortest_route(src, dest): computes shortest route from src to dest
    '''
    def __init__(self, intersections: Dict[Intersection, List[Intersection]]):
        self.intersections = intersections

    def add_street(
        self, 
        int1: Intersection,
        int2: Intersection
    ):

        # connect first intersection to second
        try:
            if not int2 in self.intersections[int1]:
                self.intersections[int1].append(int2)
        except KeyError:
            self.intersections[int1] = [int2]

        # connect second intersection to first
        try:
            if not int1 in self.intersections[int2]:
                self.intersections[int2].append(int1)
        except KeyError:
            self.intersections[int2] = [int1]


    def shortest_route(
        self, 
        src: Intersection, 
        dest: Intersection
    ) -> List[Intersection]:
        '''
        computes shortest sequence of intersections from src to dest
        uses BFS since this is an unweighted graph

        Parameters
        ----------
        src : Intersection
            first intersection in route
        dest : Intersection
            last intersection in route

        Returns
        -------
        List[Intersections]
            the shortest sequence of intersections connecting src to dest
        '''
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
    '''
    contructs a map by having eebot traverse it
    uses BFS since this is an unweighted graph

    Parameters
    ----------
    bot : robot.EEbot
        our little eebot :)
    start : Intersection
        starting intersection
    heading : int
        initial heading of eebot
        NB: needs to be facing 180 degrees away from a street (black tape)

    Returns
    -------
    Map
        the map
    '''
    m = Map({})
    s = start
    curr_int = s # current intersectoin the bot is at

    # Mark all the vertices as not visited
    visited = []
 
    # Create a queue for BFS
    queue = []

    # Mark the source node as
    # visited and enqueue it
    queue.append(s)
    visited.append(s)
 
    while queue:
        # for debugging
        print(m)
        print(curr_int)
        print(heading)
        print()
 
        # dequeue next intersection to explore
        s = queue.pop(0)
        # get directions to this intersection, and the new heading once we've
        # arrived
        directions, heading = route_to_directions( 
            m.shortest_route(curr_int, s),
            heading
        )
        # go to the intersection
        bot.follow_directions(directions)
        curr_int = s
 
        # scan for streets connected to intersection
        # also get the new heading
        streets, heading = bot.scan(heading)

        for dir, street in enumerate(streets):
            if not street:
                # don't do anything if there's no street in this direction
                continue

            # if there is a street in this direction, add it to map
            new_int = curr_int + dir_to_diff[dir]
            m.add_street(curr_int, new_int)
            if new_int not in visited:
                # if we haven't visited the adjacent intersection, queue it
                queue.append(new_int)
                visited.append(new_int)
    return m
       

# maps in classroom (for testing shortest_route)
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
