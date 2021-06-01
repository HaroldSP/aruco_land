#!/usr/bin/env python
# -*- coding: utf-8 -*-

#THIS IS A NEW TEST SCRIPT RUNNING SEVERAL FUNCSIONS FROM MODULES
#PRINT DEBUG VERSION

##########DEPENDENCIES################
#import aruco_func as AR           # my custom imports
import aruco_func_test as AR       # my custom imports
import functions as F              # my custom imports

import argparse
import cv2
import cv2.aruco as aruco
import exceptions
import imutils
import math
import numpy as np
import socket
import time

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
from imutils.video import WebcamVideoStream
from pymavlink import mavutil
from simple_pid import PID

#######VARIABLES####################
id_to_find = 72 ##Aruco
marker_size = 20 #cm

takeoff_height = 4
velocity = .25

script_mode = 1  # 1 for arm and takeoff, 2 for manual LOITER to GUIDED land
ready_to_land = 0  # 1 to trigger landing

# If True, arming from RC controller, If False, arming from this script.
manualArm = False

if __name__=='__main__':
    try:
        vehicle = F.connectMyCopter()
        while vehicle != vehicle:
            print("Waiting for vehicle to connect to mavlink.")
            time.sleep(1)

        ##
        ##SETUP PARAMETERS TO ENABLE PRECISION LANDING
        ##
        vehicle.parameters['PLND_ENABLED'] = 1
        vehicle.parameters['PLND_TYPE'] = 1 ##1 for companion computer
        vehicle.parameters['PLND_EST_TYPE'] = 0 ##0 for raw sensor, 1 for kalman filter pos estimation
        vehicle.parameters['LAND_SPEED'] = 20 ##Descent speed of 20cm/s

        #DEBUG SECTION 1#
        print(vehicle.parameters['LAND_SPEED'])
        #################

        if script_mode ==1:
            #F.arm_and_takeoff(takeoff_height)
            ############2#######################
            print("FLYING UP!!", takeoff_height)
            ####################################
            print(str(time.time()))
            #F.send_local_ned_velocity(velocity,velocity,0) ##Offset drone from target
            time.sleep(1)
            ready_to_land=1
            ############3#######################
            print("READY TO LAND! Should be 1 =", ready_to_land)
            ####################################
        elif script_mode==2:
            while vehicle.mode!='GUIDED':
                time.sleep(1)
                print("Waiting for manual change from mode "+str(vehicle.mode)+" to GUIDED")
            ready_to_land=1

        if ready_to_land==1:
            ############4#######################
            print("ENTERING ready_to_land SECTION")
            ARMED = 0
            print(ARMED)
            ####################################
            #while vehicle.armed==True:
            #while ARMED != 10:
            while True:
                AR.lander()
                time.sleep(1)
                ARMED = ARMED + 1
                print(ARMED)
            print('waited 10 sec to land')
            print('LANDED')
            end_time = time.time()
            total_time=end_time-start_time
            total_time=abs(int(total_time))

            total_count=found_count+notfound_count
            freq_lander=total_count/total_time
            print("Total iterations: "+str(total_count))
            print("Total seconds: "+str(total_time))
            print("------------------")
            print("lander function had frequency of: "+str(freq_lander))
            print("------------------")
            print("Vehicle has landed")
            print("------------------")
    except:
        pass
