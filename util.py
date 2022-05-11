import json
import map
from typing import Dict, Any

def map_to_dict(m: map.Map) -> Dict[Any, Any]:
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

def dict_to_map(dictionary: Dict[Any, Any]) -> map.Map:
    '''converts json serializable dictionary to map object'''
    m = map.Map([])
    for inter_dict in dictionary['intersections']:
        i = map.Intersection(tuple(inter_dict['coords']))
        i.streets = list(inter_dict['streets'])
        m.intersections.append(i)
    return m

def save_map(m: map.Map, filename: str):
    '''saves map to local json file'''
    with open(filename, 'w') as f:
        f.write(json.dumps(map_to_dict(m)))

def load_map(filename: str) -> map.Map:
    '''loads map from local json file'''
    with open(filename, 'r') as f:
        data = f.read()
    return dict_to_map(json.loads(data))
