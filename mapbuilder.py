from typing import Tuple
import robot
from map import Intersection, Map, route_to_directions

# cardinal directions
N = 0
W = 1
S = 2
E = 3

# directions
F = 0
L = 1
B = 2
R = 3

def goto(
    m: Map,
    bot: robot.EEBot, 
    heading: int, 
    start: Intersection, 
    dest: Intersection
) -> int:
    '''
    sends eebot from start to dest

    Parameters
    ----------
    m : Map
        map eebot is traversing
    bot : robot.EEbot
        the bot :)
    heading : int
        initial heading of eebot
    start : Intersection
        (long, lat) starting point
    dest : Intersection
        (long, lat) ending point

    Returns
    -------
    int
        new heading
    '''
    route = m.shortest_route(start, dest)
    directions, heading = route_to_directions(route, heading)
    bot.follow_directions(directions)
    return heading

def build_map(bot: robot.EEBot, heading: int, start: Tuple[int,int] = (0, 0)):
    '''
    contructs a map by having eebot traverse it

    Parameters
    ----------
    bot : robot.EEbot
        the bot :)
    heading : int
        initial heading of eebot
        NB: needs to be facing 180 degrees away from a street (black tape)
    start : Tuple[int, int], optional
        (long, lat) starting point (default is (0, 0))

    Returns
    -------
    Map
        the map
    '''
    m = Map()
    current = Intersection(start)
    come_back = [current]

    while come_back:
        if -1 not in current.streets:
            # know everything about this intersection already
            # so got to an unknown intersection
            next = come_back.pop(0)
            heading = goto(m, bot, heading, current, next)
            current = next

        # if the street to the right is unkown, check right in scan
        # otherwise, check left in scan
        check_right = current.streets[(heading + R)%4] == -1

        street_info, heading = bot.partial_scan(check_right, heading)

        # update streets with new info
        current.streets = [
            street_info[d]
            for d in range(len(current.streets))
            if current.streets[d] == -1
        ]

        # update neighbors as well
        for neighbor_coords in current.neighbors():
            neighbor = m.get_intersection(neighbor_coords)

            if neighbor:
                # this neighbor is already in map, so update streets
                neighbor.streets[neighbor.direction(current)] = 1

                # remove neighbor from come_back queue if necessary
                if -1 not in neighbor.streets and neighbor in come_back:
                    come_back.remove(neighbor)
            else:
                # neighbor not yet in map, so initialize it
                neighbor = Intersection(neighbor_coords)
                neighbor.streets[neighbor.direction(current)] = 1

                # we'll need to come back to this later
                come_back.append(neighbor)

        # next intersection to explore is at the top of the come_back queue
        next = come_back.pop(0)
        heading = goto(m, bot, heading, current, next)
        current = next
