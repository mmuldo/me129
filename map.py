from __future__ import annotations
from typing import Dict, List, Optional, Tuple, Union
import robot

# Cardinal directions
N = 0 #north
W = 1 #west
S = 2 #south
E = 3 #east

# Directions
F = 0   # forwards
L = 1   # left
B = 2   # backwards
R = 3   # right

# street existence
UNKNOWN = 0
PRESENT = 1
ABSENT = -1

class Intersection:
    '''
    Represents an intersection in a map
    basically just a tuple (longitude, latitude)

    Attributes
    ----------
    coords : Tuple[int, int]
        (longitude, latitude) in map
    streets : List[int]
        presence of [N, W, S, E] streets
        -1 --> unknown
        0 --> no street
        1 --> street exists
    '''
    def __init__(self, coords: Tuple[int, int]):
        self.coords = coords
        self.streets = [-1] * 4

    def neighbors(self) -> List[Tuple[int, int]]:
        '''
        returns (long, lat) known neighbors of this intersection
        '''
        neighs = []
        if self.streets[N] == PRESENT:
            neighs.append((self.coords[0], self.coords[1] + 1))
        if self.streets[W] == PRESENT:
            neighs.append((self.coords[0] - 1, self.coords[1]))
        if self.streets[S] == PRESENT:
            neighs.append((self.coords[0], self.coords[1] - 1))
        if self.streets[E] == PRESENT:
            neighs.append((self.coords[0] + 1, self.coords[1]))
        return neighs

    def __eq__(self, other):
        if isinstance(other, Intersection):
            return self.coords == other.coords
        elif isinstance(other, tuple):
            return self.coords == other
        return False

    def __hash__(self):
        return self.coords.__hash__()

    #def __add__(self, other):
    #    return Intersection(self.long + other.long, self.lat + other.lat)

    #def __sub__(self, other):
    #    return Intersection(self.long - other.long, self.lat - other.lat)

    def __str__(self):
        return str(self.coords)

def route_to_directions(
    route: List[Tuple[int, int]],
    heading: int
) -> Tuple[List[int], int]:
    '''
    converts a series of Intersections (i.e. coordinate points) to directions
    to follow

    Parameters
    ----------
    route : List[Tuple[int, int]]
        sequence of Intersections in map to get from one
        intersection to another
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

    print([str(i) for i in route])
    print(dirs)
    return (dirs, curr_heading)




class Map:
    '''
    represents map (graph in which the nodes are intersections and edges are
    streets)

    Attributes
    ----------
    intersections : List[Intersection]
        adjacency list for nodes in graph
        intersections' streets attribute indicates info about neighbours

    Methods
    -------
    add_street(int1, int2): adds street (edge) to map
    shortest_route(src, dest): computes shortest route from src to dest
    '''
    def __init__(self, intersections: List[Intersection] = []):
        self.intersections = intersections

    def get_intersection(
        self, coords: Tuple[int, int]
    ) -> Union[Intersection, None]:
        '''
        converts coords to intersection in map (if it exists)
        if it doesn't exist, returns None

        Parameters
        ----------
        coords : Tuple[int, int]
            (long, lat)

        Returns
        -------
        Union[Intersection, None]
            if coords are in map, returns corresponding intersection
            otherwise, returns None
        '''
        for intersection in self.intersections:
            if intersection.coords == coords:
                return intersection
        return None


    def neighbors(self, intersection: Intersection) -> List[Intersection]:
        '''
        returns neighbors of specified intersection

        Parameters
        ----------
        intersection : Intersection
            intersection in map

        Preconditions
        -------------
        assumes intersection is in map

        Returns
        -------
        List[Intersection]
            list of neighbors
        '''
        assert intersection in self.intersections

        return [
            self.get_intersection(coords)
            for coords in intersection.neighbors()
        ]

    def is_valid(self) -> bool:
        '''
        checks that map is valid, i.e. all neighbors of intersections
        are themselves connected to said intersections

        Returns
        -------
        bool
            True --> valid
            False --> something is wrong
        '''
        for intersection1 in self.intersections:
            for intersection2 in self.neighbors(intersection1):
                if intersection1 not in self.neighbors(intersection2):
                    return False
        return True

    def shortest_route(
        self,
        src: Intersection,
        dest: Intersection
    ) -> List[Intersection]:
        '''
        computes shortest sequence of intersections from src to dest
        uses dijkstra

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
        pass

    def __str__(self):
        pass

def build_map(bot: robot.EEBot, start: Intersection, heading: int):
    '''
    contructs a map by having eebot traverse it

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
    pass
