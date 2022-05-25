import util
import json
import mapbuilder
import robot
import map
from typing import Tuple

# cardinal directions
N = 0
W = 1
S = 2
E = 3


# maps
square = util.load_map('square.json')
map_l1p = util.load_map('map_l1p.json')

def demo_goto(
    m: map.Map,
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
    return mapbuilder.goto(
        m,
        bot,
        heading,
        s,
        d
    )

def demo_build_map() -> Tuple[map.Map, int]:
    '''
    contructs a map by having eebot traverse it.
    returns to origin once done.
    assumes facing north and at (0, 0) to start.
    just runs build_map

    Returns
    -------
    (Map, int)
        the map and the resulting heading after process is completed
    '''
    return mapbuilder.build_map(bot, 0, (0, 0))
