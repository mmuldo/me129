from util import *
import robot
import map

from typing import Tuple, List

import pigpio

########################
### cleanup function ###
########################
def cleanup(control: robot.EEBot):
    ''' shuts down necessary things '''
    control.ultra.shutdown_ultrasonic()
    control.shutdown()

######################
### robot.py tests ###
######################
class TestAssesBlockage:
    '''tests asses_blockage in EEBot class'''
    @staticmethod
    def base_test(map_filename: str, coords: Tuple[int, int], heading: int):
        '''base test function'''
        try:
            # load map from json
            m = map.load_map(map_filename)
            # get origin from map
            i = m.get_intersection(coords)
            # setup eebot
            control = robot.EEBot(
                passed_io=pigpio.pi(),
                botMap=m,
                intersection=i,
                heading=heading,
                come_back=[]
            )
            # Dylan did this
            # inter = map.Intersection([0,0],False,[1,-1,-1,-1])
            # control.goto(inter)
           
            print('ASSESS BLOCKAGE TEST')
            # 'True' -> map didn't change
            # 'False' -> map changed
            print('assess_blockage map same: ', control.assess_blockage())
            # get a visual on the map
            control.map.visualize()
            print()
        except BaseException as ex:
            print('exception while testing assess_blockage')
            cleanup(control)
            print('traceback:')
            raise ex


    @staticmethod
    def square_origin_N():
        '''square map, at origin, facing N'''
        TestAssesBlockage.base_test('maps/square.json', (0,0), N)

    @staticmethod
    def square_11_W():
        '''square map, at (1,1), facing W'''
        TestAssesBlockage.base_test('maps/square.json', (1,1), W)

    @staticmethod
    def l2_21_E():
        '''l2 map, at (2,1), facing E'''
        TestAssesBlockage.base_test('maps/map_l2.json', (2,1), E)

    @staticmethod
    def l2_32_S():
        '''l2 map, at (3,2), facing S'''
        TestAssesBlockage.base_test('maps/map_l2.json', (3,2), S)



class TestNextIntersection:
    '''tests next_intersection in EEBot class'''
    @staticmethod
    def base_test(map_filename: str, coords: Tuple[int, int], heading: int):
        '''base test function'''
        try:
            # load map from json
            m = map.load_map(map_filename)
            # get origin from map
            i = m.get_intersection(coords)
            # setup eebot
            control = robot.EEBot(
                passed_io=pigpio.pi(),
                botMap=m,
                intersection=i,
                heading=heading,
                come_back=[]
            )

            print('NEXT INTERSECITON TEST:')
            # 'True' -> succeeded in reaching next intersection
            # 'False' -> aborted and turned around
            print('didnt abort: ', control.next_intersection())
            print('resulting intersection: ', control.intersection)
            print('resulting heading: ', control.heading)
            # get a visual on the map
            control.map.visualize()
            print()
        except BaseException as ex:
            print('exception while testing next_intersection')
            cleanup(control)
            print('traceback:')
            raise ex

    @staticmethod
    def square_origin_N():
        '''square map, at origin, facing N'''
        TestNextIntersection.base_test('maps/square.json', (0,0), N)

    @staticmethod
    def square_11_W():
        '''square map, at (1,1), facing W'''
        TestNextIntersection.base_test('maps/square.json', (1,1), W)

    @staticmethod
    def l2_21_E():
        '''l2 map, at (2,1), facing E'''
        TestNextIntersection.base_test('maps/map_l2.json', (2,1), E)

    @staticmethod
    def l2_32_S():
        '''l2 map, at (3,2), facing S'''
        TestNextIntersection.base_test('maps/map_l2.json', (3,2), S)


class TestTurn:
    '''tests turn in EEBot class'''
    @staticmethod
    def base_test(
        map_filename: str,
        coords: Tuple[int, int],
        heading: int,
        direction: int
    ):
        '''base test function'''
        try:
            # load map from json
            m = map.load_map(map_filename)
            # get origin from map
            i = m.get_intersection(coords)
            # setup eebot
            control = robot.EEBot(
                passed_io=pigpio.pi(),
                botMap=m,
                intersection=i,
                heading=heading,
                come_back=[]
            )

            print('TURN TEST:')
            # 'True' -> map same after turn
            # 'False' -> map not same before/after turn
            #   if not same before, shouldn't have done turn
            print('didnt abort: ', control.turn(direction))
            print('resulting heading: ', control.heading)
            # get a visual on the map
            control.map.visualize()
            print()
        except BaseException as ex:
            print('exception while testing turn')
            cleanup(control)
            print('traceback:')
            raise ex

    @staticmethod
    def square_origin_N_R():
        '''square map, at origin, facing N, turn R'''
        TestTurn.base_test('maps/square.json', (0,0), N, R)

    @staticmethod
    def square_origin_N_F():
        '''square map, at origin, facing N, turn F'''
        TestTurn.base_test('maps/square.json', (0,0), N, F)

    @staticmethod
    def square_11_W_L():
        '''square map, at (1,1), facing W, turn L'''
        TestTurn.base_test('maps/square.json', (1,1), W, L)

    @staticmethod
    def l2_21_N_B():
        '''l2 map, at (2,1), facing N, turn B'''
        TestTurn.base_test('maps/map_l2.json', (2,1), N, B)

    @staticmethod
    def l2_32_S_R():
        '''l2 map, at (3,2), facing S, turn R'''
        TestTurn.base_test('maps/map_l2.json', (3,2), S, R)


class TestFollowDirections:
    '''tests follow_directions in EEBot class'''
    @staticmethod
    def base_test(
        map_filename: str,
        coords: Tuple[int, int],
        heading: int,
        directions: List[int]
    ):
        '''base test function'''
        try:
            # load map from json
            m = map.load_map(map_filename)
            # get origin from map
            i = m.get_intersection(coords)
            # setup eebot
            control = robot.EEBot(
                passed_io=pigpio.pi(),
                botMap=m,
                intersection=i,
                heading=heading,
                come_back=[]
            )

            print('FOLLOW DIRECTIONS TEST:')
            # 'True' -> made it to destination without map changing
            # 'False' -> map changed while following directions
            #   if not same before, shouldn't have done turn
            print('didnt abort: ', control.follow_directions(directions))
            print('resulting intersection: ', control.intersection)
            print('resulting heading: ', control.heading)
            # get a visual on the map
            control.map.visualize()
            print()
        except BaseException as ex:
            print('exception while testing follow_directions')
            cleanup(control)
            print('traceback:')
            raise ex

    @staticmethod
    def square_origin_S_BRBL():
        '''square map, at origin, facing S
        turns B, R, B, L such that if completed we're back where we started
        '''
        TestFollowDirections.base_test(
            'maps/square.json',
            (0,0),
            N,
            [B, R, B, L]
        )

    @staticmethod
    def l2_11_N_FRFRRLRR():
        '''l2 map, at (1,1), facing N
        turns F, R, F, R, R, L, R, R such that if completed we're back
        where we started
        '''
        TestFollowDirections.base_test(
            'maps/map_l2.json',
            (1,1),
            N,
            [F, R, F, R, R, L, R, R]
        )


class TestGoto:
    '''tests goto in EEBot class'''
    @staticmethod
    def base_test(
        map_filename: str,
        coords: Tuple[int, int],
        heading: int,
        dest: Tuple[int, int]
    ):
        '''base test function'''
        try:
            # load map from json
            m = map.load_map(map_filename)
            # get origin from map
            i = m.get_intersection(coords)
            # get destination intersection from map
            dest_int = m.get_intersection(dest)
            # setup eebot
            control = robot.EEBot(
                passed_io=pigpio.pi(),
                botMap=m,
                intersection=i,
                heading=heading,
                come_back=[]
            )

            print('GOTO TEST:')
            # 'True' -> made it to destination
            # 'False' -> gave up because no way to get to destination
            print('didnt give up: ', control.goto(dest_int, visualize=True))
            print('resulting intersection: ', control.intersection)
            print('resulting heading: ', control.heading)
            # get a visual on the map
            control.map.visualize()
            print()
        except BaseException as ex:
            print('exception while testing goto')
            cleanup(control)
            print('traceback:')
            raise ex

    @staticmethod
    def square_origin_N_11():
        '''square map, at origin, facing N, destination (1,1)'''
        TestGoto.base_test('maps/square.json', (0,0), N, (1,1))

    @staticmethod
    def square_origin_N_10():
        '''square map, at origin, facing N, destination (1,0)'''
        TestGoto.base_test('maps/square.json', (0,0), N, (1,0))

    @staticmethod
    def l2_origin_N_30():
        '''l2 map, at origin, facing N, destination (3,0)'''
        TestGoto.base_test('maps/map_l2.json', (0,0), N, (3,0))

    @staticmethod
    def l2_30_S_02():
        '''l2 map, at (3,0), facing S, destination (0,2)'''
        TestGoto.base_test('maps/map_l2.json', (3,0), S, (0,2))



class TestPartialScan:
    '''tests partial_scan in EEBot class'''
    @staticmethod
    def base_test(
        map_filename: str,
        coords: Tuple[int, int],
        heading: int,
        check_right: bool
    ):
        '''base test function'''
        try:
            # load map from json
            m = map.load_map(map_filename)
            # get origin from map
            i = m.get_intersection(coords)
            # get destination intersection from map
            dest_int = m.get_intersection(dest)
            # setup eebot
            control = robot.EEBot(
                passed_io=pigpio.pi(),
                botMap=m,
                intersection=i,
                heading=heading,
                come_back=[]
            )

            print('PARTIAL SCAN TEST:')
            # returns [N, W, S, E] int list indicating status of each street
            print('street info: ', control.partial_scan(check_right))
            print('resulting heading: ', control.heading)
            # get a visual on the map
            control.map.visualize()
            print()
        except BaseException as ex:
            print('exception while testing partial_scan')
            cleanup(control)
            print('traceback:')
            raise ex

    @staticmethod
    def square_origin_S_checkR():
        '''square map, at origin, facing S, check right'''
        TestPartialScan.base_test('maps/square.json', (0,0), S, True)

    @staticmethod
    def square_origin_S_checkL():
        '''square map, at origin, facing S, check left'''
        TestPartialScan.base_test('maps/square.json', (0,0), S, False)

    @staticmethod
    def l2_21_N_checkR():
        '''l2 map, at (2,1), facing N, check right'''
        TestPartialScan.base_test('maps/map_l2.json', (2,1), N, True)

    @staticmethod
    def l2_21_N_checkL():
        '''l2 map, at (2,1), facing N, check left'''
        TestPartialScan.base_test('maps/map_l2.json', (2,1), N, False)


class TestFind:
    '''tests find in EEBot class'''
    @staticmethod
    def base_test(control: robot.EEBot, dest: Tuple[int,int]):
        '''base test function'''
        try:
            print('FIND TEST:')
            control.find(dest, True)
            print()
        except BaseException as ex:
            print('exception while testing find')
            cleanup(control)
            print('traceback:')
            raise ex

    @staticmethod
    def square_origin_S_11():
        '''square map, at origin, facing S, 
        find (1,1)'''
        control = robot.EEBot(pigpio.pi())
        TestFind.base_test(control, (1,1))

    @staticmethod
    def square_origin_S_11_then_origin():
        '''square map, at origin, facing S,
        find (1,1) then find origin'''
        control = robot.EEBot(pigpio.pi())
        TestFind.base_test(control, (1,1))
        TestFind.base_test(control, (0,0))

    @staticmethod
    def square_origin_S_11_then_10():
        '''square map, at origin, facing S,
        find (1,1) then find (1,0)'''
        control = robot.EEBot(pigpio.pi())
        TestFind.base_test(control, (1,1))
        TestFind.base_test(control, (1,0))

    @staticmethod
    def l1p_origin_S_22():
        '''l1p map, at origin, facing S,
        find (2,2)'''
        control = robot.EEBot(pigpio.pi())
        TestFind.base_test(control, (2,2))

    @staticmethod
    def l1p_origin_S_22_then_20():
        '''l1p map, at origin, facing S,
        find (2,2) then find (2,0)'''
        control = robot.EEBot(pigpio.pi())
        TestFind.base_test(control, (2,2))
        TestFind.base_test(control, (2,0))

    @staticmethod
    def l1p_origin_S_22_then_20_then_01():
        '''l1p map, at origin, facing S,
        find (2,2) then find (2,0) then find (0, 1)'''
        control = robot.EEBot(pigpio.pi())
        TestFind.base_test(control, (2,2))
        TestFind.base_test(control, (2,0))
        TestFind.base_test(control, (0,1))

    @staticmethod
    def l1p_origin_S_22_then_20_then_01_then_30():
        '''l1p map, at origin, facing S,
        find (2,2) then find (2,0) then find (0, 1), then find (3, 0)'''
        control = robot.EEBot(pigpio.pi())
        TestFind.base_test(control, (2,2))
        TestFind.base_test(control, (2,0))
        TestFind.base_test(control, (0,1))
        TestFind.base_test(control, (3,0))
