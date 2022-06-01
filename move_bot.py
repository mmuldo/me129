#!/usr/bin/env python3
'''
driver for eebot
'''

# project libraries
from util import *
import map
import robot
import tests

# utility libraries
import time
import json
import random

# other libraries
import pigpio

def driver():
    '''driver function (put main code here)'''
    control = robot.EEBot(pigpio.pi())
    try:
        # stuff goes here
        pass
    except BaseException as ex:
        print('uh oh...theres an exception')
        control.ultra.shutdown_ultrasonic()
        control.shutdown()
        print('heres the traceback:')
        raise ex

if __name__ == "__main__":
    # commented out, because I put the tests here
    # driver()


    #############
    ### TESTS ###
    #############
    # peform each one one at a time.
    # for each test, I recommend:
    #   1. testing without any obstacles (to make sure stuff still works)
    #   2. testing multiple times, each time with a different obstacle
    #       configuration
    # MAKE SURE YOU CHECK THE LEDS ARE CALIBRATED BEFORE STARTING.

    #####################
    ## assess_blockage ##
    #####################
    #tests.TestAssesBlockage.square_origin_N()
    #tests.TestAssesBlockage.square_11_W()
    #tests.TestAssesBlockage.l2_21_E()
    #tests.TestAssesBlockage.l2_32_S()

    #######################
    ## next_intersection ##
    #######################
    #tests.TestNextIntersection.square_origin_N()
    #tests.TestNextIntersection.square_11_W()
    #tests.TestNextIntersection.l2_21_E()
    #tests.TestNextIntersection.l2_32_S()

    ##########
    ## turn ##
    ##########
    #tests.TestTurn.square_origin_N_R()
    #tests.TestTurn.square_origin_N_F()
    #tests.TestTurn.square_11_W_L()
    #tests.TestTurn.l2_21_N_B()
    #tests.TestTurn.l2_32_S_R()

    #######################
    ## follow_directions ##
    #######################
    tests.TestFollowDirections.square_origin_S_BRBL()
    #tests.TestFollowDirections.l2_11_N_FRFRRLRR()

    #tests.list_threads()

    ##########
    ## goto ##
    ##########
    #tests.TestGoto.square_origin_N_11()
    #tests.TestGoto.square_origin_N_10()
    #tests.TestGoto.l2_origin_N_30()
    #tests.TestGoto.l2_30_S_02()

    ##################
    ## partial_scan ##
    ##################
    #tests.TestPartialScan.square_origin_S_checkR()
    #tests.TestPartialScan.square_origin_S_checkL()
    #tests.TestPartialScan.l2_21_N_checkR()
    #tests.TestPartialScan.l2_21_N_checkL()

    ##########
    ## find ##
    ##########
    #tests.TestFind.square_origin_S_11()
    #tests.TestFind.square_origin_S_11_then_origin()
    #tests.TestFind.square_origin_S_11_then_10()
    #tests.TestFind.l1p_origin_S_22()
    #tests.TestFind.l1p_origin_S_22_then_20()
    #tests.TestFind.l1p_origin_S_22_then_20_then_01()
    #tests.TestFind.l1p_origin_S_22_then_20_then_01_then_30()
