from __future__ import annotations
'''
black-tape map abstraction: for eebot to roam around

Classes
-------
Intersection: up to 4-way intersection in map
Map: graph where nodes are Intersections and edges are streets
    connecting intersections

Functions
---------
route_to_directions(route, heading): converts a series of Intersections
    to directions to follow
map_to_dict(m): converts map to dictionary (for json serializable)
dict_to_map(dictionary): converts dictionary to map object
save_map(m, filename): saves map to json file
load_map(filename): loads map from json file
'''

# project libraries
from util import *

# utility libraries
from typing import Dict, List, Optional, Tuple, Union, Any
import json
import random
import sys

# other libraries
import matplotlib.pyplot as plt


###############
### classes ###
###############
class Intersection:
    '''
    Represents an intersection in a map
    basically just a tuple (longitude, latitude)

    Attributes
    ----------
    coords : Tuple[int, int]
        (longitude, latitude) in map
    blocked : bool
        True --> intersection blocked
        False --> intersection accessible
    streets : List[int]
        presence of [N, W, S, E] streets
        -1 --> unknown
        0 --> no street
        1 --> street exists
        2 --> street is blocked by an obstacle

    Methods
    -------
    copy(): creates a copy of this object
    visualize(): uses matplotlib to visualize the intersection
    neighbors(): gets the intersection's accessible neighbors (list of 
        coordinate tuples)
    blocked_neighbors(): same as neighbors, but for inaccessible neighbors
    direction(neighbor): computes the direction (N, W, S, E) that the
        neighbor is in
    distance(dest): computes distance from self to dest
    '''
    def __init__(
        self, 
        coords: Tuple[int, int], 
        blocked: bool = False,
        streets: List[int] = [-1] * 4
    ):
        '''
        Parameters
        ----------
        coords : Tuple[int, int]
            (longitude, latitude) in map
        blocked : bool
            True --> intersection blocked
            False --> intersection accessible
        streets : List[int]
            presence of [N, W, S, E] streets
            -1 --> unknown
            0 --> no street
            1 --> street exists
            2 --> street is blocked by an obstacle
        '''
        self.coords = coords
        self.blocked = blocked
        self.streets = streets            
       
    #############
    ## utility ##
    #############
    def __str__(self):
        '''string representation'''
        return str(self.coords)

    def __hash__(self):
        '''how to distinguish this from other dicitonary keys'''
        # in this case, just use the hash of the coords tuple
        return self.coords.__hash__()

    def __eq__(self, other):
        '''indicates how [int1] == [int2] should be evaluated'''
        if isinstance(other, Intersection):
            # if being compared to another intersection, the two are equal
            #   if and only if all of their attributes are equal
            return (
                self.coords == other.coords and
                self.streets == other.streets and
                self.blocked == other.blocked
            )
        elif isinstance(other, tuple):
            # if being compared to a coordinate tuple, the two are equal
            #   if and only if intersection's coords are equal to tuple
            return self.coords == other
        return False

    def copy(self) -> Intersection:
        '''creates a copy of this object'''
        return Intersection(self.coords, self.blocked, self.streets)

    def visualize(self):
        '''
        plots intersection on a matplotlib graph as follows:
            point placed at coords:
                black --> accessible
                red --> blocked
            line connecting intersection to neighbors:
                absent --> absent
                gray --> unknown
                black --> present
                red --> blocked
        '''
        # plot point at coords
        plt.plot(
            self.coords[0], 
            self.coords[1], 
            marker = 'o', 
            ms = 10,
            color = 'red' if self.blocked else 'black'
        )

        for dir, status in enumerate(self.streets):
            if not status:
                # street doesn't exist, so don't plot anything
                continue

            diff = dir_to_diff[dir]
            # get adjacent coords
            adjacent_coords = (
                self.coords[0] + diff[0],
                self.coords[1] + diff[1]
            )

            color = 'black'
            if status == UNKNOWN:
                color = 'gray'
            elif status == BLOCKED:
                color = 'red'

            plt.plot(
                [self.coords[0]] + [adjacent_coords[0]],
                [self.coords[1]] + [adjacent_coords[1]],
                color = color
            )

    ###############
    ## neighbors ##
    ###############
    def neighbors(self) -> List[Tuple[int, int]]:
        '''
        returns (long, lat) known, accessible neighbors of this intersection
        '''
        neighs = []
        if self.streets[N] == PRESENT:
            neighs.append((self.coords[0], self.coords[1] + 1))
        else:
            neighs.append(ABSENT)
        if self.streets[W] == PRESENT:
            neighs.append((self.coords[0] - 1, self.coords[1]))
        else:
            neighs.append(ABSENT)
        if self.streets[S] == PRESENT:
            neighs.append((self.coords[0], self.coords[1] - 1))
        else:
            neighs.append(ABSENT)
        if self.streets[E] == PRESENT:
            neighs.append((self.coords[0] + 1, self.coords[1]))
        else:
            neighs.append(ABSENT)
            #DYLAN - I added ABSENT if there is no intersection
        return neighs

    def blocked_neighbors(self) -> List[Tuple[int, int]]:
        '''
        returns (long, lat) known, blocked neighbors of this intersection
        '''
        neighs = []
        if self.streets[N] == BLOCKED:
            neighs.append((self.coords[0], self.coords[1] + 1))
        else:
            neighs.append(ABSENT)
        if self.streets[W] == BLOCKED:
            neighs.append((self.coords[0] - 1, self.coords[1]))
        else:
            neighs.append(ABSENT)
        if self.streets[S] == BLOCKED:
            neighs.append((self.coords[0], self.coords[1] - 1))
        else:
            neighs.append(ABSENT)
        if self.streets[E] == BLOCKED:
            neighs.append((self.coords[0] + 1, self.coords[1]))
        else:
            neighs.append(ABSENT)
        return neighs

    
    #############
    ## vectors ##
    #############
    def direction(self, neighbor: Intersection) -> int:
        '''
        computes direction of neighbor in relation to self

        Parameters
        ----------
        neighbor : Intersection
            neighbor intersection

        Returns
        -------
        int
            N, W, S, or E
        '''
        print('coords')
        print(neighbor.coords)
        print(self.coords)
        #DYLAN SKIP THIS HERE
        # if(self.coords == 0):
        #     #dylan hack
        #     return (1,0)
        #MATT HERE
        return diff_to_dir[(
            neighbor.coords[0] - self.coords[0],
            neighbor.coords[1] - self.coords[1]
        )]

    def distance(self, dest: Tuple[int, int]) -> float:
        '''
        computes distance from self.coords to dest

        Parameters
        ----------
        dest : Tuple[int, int]
            (long, lat) coords
        
        Returns
        -------
        float
            distance from self to dest
        '''
        coords = self.coords
        xsquare = (dest[0] - coords[0])**2
        ysquare = (dest[1] - coords[1])**2
        return (xsquare + ysquare)**0.5




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
    copy(): creates a copy of this object
    visualize(): visualize map using maplotlib
    shortest_route(src, dest): computes shortest route from src to dest
    get_intersection(coords): gets the intersection based on coordinate
        tuple, if it exists in the map
    neighbors(intersection): gets the intersection's accessible neighbors 
        (list of Intersections)
    blocked_neighbors(): same as neighbors, but for inaccessible neighbors
    get_neighbor(intersection, direction): gets the neighbor of specified
        intersection in specified direction
    is_valid(): verifies validity of map
    shortest_route(src, dest): computes shortest route from src to dest
        using dijkstra
    '''
    def __init__(self, intersections: List[Intersection]):
        '''
        Parameters
        ----------
        intersections : List[Intersection]
            adjacency list for nodes in graph;
            intersections' streets attribute indicates info about neighbours
        '''
        self.intersections = intersections

    #############
    ## utility ##
    #############
    def __str__(self):
        '''string representation of map in adjacency list type format'''
        s = ''
        for intersection in self.intersections:
            neighbor_list = ','.join([
                str(n) 
                for n in self.neighbors(intersection)
            ])
            s += f'{intersection}: {neighbor_list}\n'
        return s

    def __eq__(self, other: Map):
        '''
        indicates how [map1] == [map2] should be evaluated;
        in this case, two maps are equal if and only if their intersection
        lists are subsets of each other.
        '''
        # check that self.intersections is subset of other.intersections
        for self_intersection in self.intersections:
            if not self_intersection in other.intersections:
                return False
        # check that other.intersections is subset of self.intersections
        for other_intersection in other.intersections:
            if not other_intersection in self.intersections:
                return False
        return True

    def copy(self) -> Map:
        '''creates copy of this object'''
        return Map([i.copy() for i in self.intersections])

    def visualize(self):
        '''
        plots map on a matplotlib graph as follows:
            intersection points:
                black --> accessible
                red --> blocked
            street connecting intersections:
                absent --> absent
                black dotted --> unknown
                black solid --> present
                red --> blocked
        '''
        # TODO: not sure the best way to update the plot, but this seems
        #   to work
        plt.close()
        for intersection in self.intersections:
            intersection.visualize()
        plt.draw()
        plt.pause(0.01)

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
        for intersection1 in self.intersections:
            for intersection2 in self.blocked_neighbors(intersection1):
                if intersection1 not in self.blocked_neighbors(intersection2):
                    return False
        return True


    ###################
    ## intersections ##
    ###################
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
        returns accessible neighbors of specified intersection

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

    def blocked_neighbors(
        self, 
        intersection: Intersection
    ) -> List[Intersection]:
        '''
        returns blocked neighbors of specified intersection

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
            list of blocked neighbors
        '''
        assert intersection in self.intersections

        return [
            self.get_intersection(coords)
            for coords in intersection.blocked_neighbors()
        ]

    def get_neighbor(
        self, 
        intersection: Intersection, 
        direction: int
    ) -> Union[Intersection, None]:
        '''
        gets the neighbor of the intersection in the specified direction

        Parameters
        ----------
        intersection : Intersection
            the intersection to get the neighbor of
        direction : int
            N, W, S, E direction

        Returns
        -------
        Union[Intersection, None]
            returns the neighboring Intersection if it exists, and None if
            intersection has no neighbor in direction
        '''
        # get vector difference
        diff = dir_to_diff[direction]

        # get hypothetical neighbor (could be None)
        neighbor = self.get_intersection((
            diff[0] + intersection.coords[0],
            diff[1] + intersection.coords[1]
        ))

        # get list of valid neighbors
        neighbors = self.neighbors(intersection)

        # return neighbor if it truly is the neighbor of intersection
        if neighbor in neighbors:
            return neighbor

        # otherwise return none
        return None


    ################
    ## navigation ##
    ################
    def shortest_route(
        self,
        src: Intersection,
        dest: Intersection
    ) -> List[Intersection]:
        '''
        computes shortest sequence of intersections from src to dest.
        uses dijkstra.

        Parameters
        ----------
        src : Intersection
            first intersection in route
        dest : Intersection
            last intersection in route

        Returns
        -------
        List[Intersections]
            the shortest sequence of intersections connecting src to dest.
            if there is no such route due to blockage, returns an empty
            list.
        '''
        def min_dist_intersection(
            dist: Dict[Intersection, int],
            visited: Dict[Intersection, bool]
        ) -> Intersection:
            '''
            utility function for shortest_route.
            picks the next closest unblocked intersection to src that has 
            not already been visited.

            Parameters
            ----------
            dist : Dict[Intersection, int]
                maps intersections to their distance from src
            visited : Dict[Intersection, int]
                maps intersections to whether or not they've been visited

            Returns
            -------
            Intersection
                the closest unblocked unvisited Intersection to src
            '''
            # initialize min distance to max possible value
            min = sys.maxsize
            # min intersection is initially randomly chosen from unvisited
            # intersections
            min_intersection = random.choice(
                [
                    intersection
                    for intersection in self.intersections
                    if not visited[intersection]
                ]
            )

            for intersection in self.intersections:
                if dist[intersection] < min and not visited[intersection]:
                    # new min chosen if it's closer than previous min
                    min = dist[intersection]
                    min_intersection = intersection
            return min_intersection


        # shortest routes from src to each intersection
        routes = {
            intersection: []
            for intersection in self.intersections
        }

        # shortest distance from src to each intersection
        dist = {
            intersection: sys.maxsize
            for intersection in self.intersections
        }

        # whether or not each intersection has been visited
        visited = {
            intersection: False
            for intersection in self.intersections
        }

        # initialize src
        dist[src] = 0
        routes[src] = [src]

        while False in visited.values():
            # Pick the minimum distance intersection from
            # the set of intersections not yet processed.
            # x is always equal to src in first iteration
            x = min_dist_intersection(dist, visited)

            # Put the minimum distance intersection in the
            # shortest path tree
            visited[x] = True

            # Update dist value of neighbors
            # of the picked intersection only if the current
            # distance is greater than new distance and
            # the intersection in not in the shortest path tree
            
            for neighbor in self.neighbors(x):
                #Dylan - case of if no neighbor
                if neighbor is None:
                    continue
                if (
                    not neighbor.blocked and 
                    not visited[neighbor] and 
                    dist[neighbor] > dist[x] + 1
                ):
                    dist[neighbor] = dist[x] + 1
                    routes[neighbor] = routes[x] + [neighbor]

        return routes[dest]




#################
### functions ###
#################

###################
## map following ##
###################
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
        direction = route[j].direction(route[j+1])
        dirs.append((direction - curr_heading) % 4)
        curr_heading = direction

    return (dirs, curr_heading)


##################
## map file i/o ##
##################
def map_to_dict(m: Map) -> Dict[Any, Any]:
    '''converts map object to json serializable dictionary'''
    return {
        'intersections': [
            {
                'coords': inter.coords,
                'streets': inter.streets
            }
            for inter in m.intersections
        ]
    }

def dict_to_map(dictionary: Dict[Any, Any]) -> Map:
    '''converts json serializable dictionary to map object'''
    m = Map([])
    for inter_dict in dictionary['intersections']:
        print('inter')
        print(inter_dict['coords'])
        i = Intersection(tuple(inter_dict['coords']))
        i.streets = list(inter_dict['streets'])
        m.intersections.append(i)
    return m

def save_map(m: Map, filename: str):
    '''saves map to local json file'''
    with open(filename, 'w') as f:
        f.write(json.dumps(map_to_dict(m), indent=4))

def load_map(filename: str) -> Map:
    '''loads map from local json file'''
    with open(filename, 'r') as f:
        data = f.read()
    return dict_to_map(json.loads(data))
