from typing import Tuple
import robot
import time
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
    change_route,
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
    # update robot's heading
    bot.heading = heading

    route = m.shortest_route(start, dest)
    directions, heading = route_to_directions(route, heading)
    bot.follow_directions(directions, change_route)

    # update robot's heading
    bot.heading = heading
    return heading

def wrapped_goto(
    m: Map,
    bot: robot.EEBot,
    change_route,
    heading: int,
    start: Tuple[int, int],
    dest: Tuple[int, int]
) -> int:
    '''
    sends eebot from start to dest
    just runs goto, but avoids the Intersection instantiation business

    Parameters
    ----------
    m : Map
        map eebot is traversing
    bot : robot.EEBot
        the robot
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
    s = m.get_intersection(start)
    d = m.get_intersection(dest)
    return goto(
        m,
        bot,
        change_route,
        heading,
        s,
        d
    )

def return_to_origin(
    m: Map,
    bot: robot.EEBot,
    heading: int,
    start: Intersection
) -> int:
    '''
    sends eebot from start back to origin

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

    Returns
    -------
    int
        new heading
    '''
    origin = m.get_intersection((0,0))
    return goto(m, bot, [False], heading, start, origin)


def dance(
    bot: robot.EEBot,
    heading: int
) -> int:
    '''
    dance for me EEBot :)

    Parameters
    ----------
    bot : robot.EEbot
        the bot :)
    heading : int
        initial heading of eebot

    Returns
    -------
    int
        new heading
    '''
    # face north
    bot.turn((N - heading)%4)

    time.sleep(0.2)
    for LR in [False, True, True, False]:
        bot.spin_kick(LR)
        bot.set_pwm(0, 0)
        time.sleep(0.2)

    total_spin = 360
    while total_spin:
        total_spin -= bot.snap(False, 0.8)
        assert total_spin >= 0

    # facing north at end
    return N

def build_map(
    bot: robot.EEBot,
    heading: int,
    start: Tuple[int,int]
) -> Tuple[Map, int]:
    '''
    contructs a map by having eebot traverse it
    returns to origin once done

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
    (Map, int)
        the map and the resulting heading after process is completed
    '''
    current = Intersection(start)
    m = Map([current])
    come_back = [current]
    first_iteration = True

    while come_back:
        # next intersection to explore is at the top of the come_back queue
        next = come_back[0]
        print(f'next is {next}')
        heading = goto(m, bot, [False], heading, current, next)
        current = next

        print(f'current intersection: {current}')
        print(f'current streets: {current.streets}')

        if first_iteration:
            street_info, heading = bot.first_scan()
            first_iteration = False
            print('first iteration')
        else:
            print('in else')
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
    heading = return_to_origin(m, bot, heading, current)
    heading = dance(bot, heading)
    return (m, heading)
