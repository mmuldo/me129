from typing import Tuple
import robot
from map import Intersection, Map, route_to_directions

# existence of streets
UNKNOWN = -1
ABSENT = 0
PRESENT = 1

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
    print(f'route: {[str(i) for i in route]}')
    directions, heading = route_to_directions(route, heading)
    print(f'directions: {directions}')
    print(f'heading: {heading}')
    bot.follow_directions(directions)
    return heading

def build_map(bot: robot.EEBot, heading: int, start: Tuple[int,int]):
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
    current = Intersection(start)
    m = Map([current])
    come_back = [current]
    first_iteration = True

    while come_back:
        # next intersection to explore is at the top of the come_back queue
        next = come_back[0]
        print(f'next is {next}')
        heading = goto(m, bot, heading, current, next)
        current = next

        print(f'current intersection: {current}')
        print(f'current streets: {current.streets}')

        #if UNKNOWN not in current.streets:
        #    print('already known')
        #    # know everything about this intersection already
        #    # so got to an unknown intersection
        #    next = come_back[0]
        #    heading = goto(m, bot, heading, current, next)
        #    current = next
        #    print(f'going to {current}')
        #    print()

        if first_iteration:
            street_info, heading = bot.first_scan()
            first_iteration = False
        else:

            # if the street to the right is unkown, check right in scan
            # otherwise, check left in scan
            check_right = current.streets[(heading + R)%4] == UNKNOWN
            street_info, heading = bot.partial_scan(check_right, heading)

        print(f'street info of {current}: {street_info}')
        print(f'new heading: {heading}')

        # update streets with new info
        for d in range(len(current.streets)):
            if current.streets[d] == UNKNOWN:
                current.streets[d] = street_info[d]

        print(f'new street info: {current.streets}')

        # update neighbors as well
        for neighbor_coords in current.neighbors():
            print(f'neighbor coords: {neighbor_coords}')
            neighbor = m.get_intersection(neighbor_coords)
            print(f'neighbor: {neighbor}')

            if neighbor:
                # this neighbor is already in map, so update streets
                neighbor.streets[neighbor.direction(current)] = PRESENT

                # remove neighbor from come_back queue if necessary
                if UNKNOWN not in neighbor.streets and neighbor in come_back:
                    come_back.remove(neighbor)
            else:
                # neighbor not yet in map, so initialize it
                neighbor = Intersection(neighbor_coords)
                neighbor.streets[neighbor.direction(current)] = PRESENT
                m.intersections.append(neighbor)

                # we'll need to come back to this later
                come_back.append(neighbor)
            print(f'neighbor: {neighbor}')
            print(f'neighbor streets: {neighbor.streets}')

        # check if we need to come back to current at some point
        come_back.remove(current)
        if UNKNOWN in current.streets:
            come_back.append(current)
        print([str(i) for i in come_back])
        print(m)

        print()
        print()
        print()
        print()
    return m
