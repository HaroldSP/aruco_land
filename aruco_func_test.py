#!/usr/bin/env python
# -*- coding: utf-8 -*-

##########DEPENDENCIES################
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

#########FUNCTIONS#################

def lander():

    global p, i, d, pid, marker_size

    ##Counters and script triggers
    found_count=0
    notfound_count=0

    first_run=0 #Used to set initial time of function to determine FPS
    start_time=0
    end_time=0

    #####Camera#######
    horizontal_res = 640
    vertical_res = 480
    cap = WebcamVideoStream(src=0, width=horizontal_res, height=vertical_res).start()

    ##################
    horizontal_fov = 62.2 * (math.pi / 180)  # Pi cam V1: 53.5 V2: 62.2
    vertical_fov = 48.8 * (math.pi / 180)  # Pi cam V1: 41.41 V2: 48.8
    ##################
    calib_path = "/home/pi/video2calibration/calibrationFiles/"
    cameraMatrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
    cameraDistortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
    ##################
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()
    ##################

    if first_run==0:
        print("First run of lander!!")
        first_run=1
        start_time=time.time()
    #print("does this happens?")
    frame = cap.read()
    #print(frame)
    frame = cv2.resize(frame,(horizontal_res,vertical_res))
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

    print("FIRST TIME LAND COMMAND - UNCOMMENT LATER!")
    print("DRONE IS IN THE LAND MODE")
    # if vehicle.mode!='LAND':
    #     vehicle.mode=VehicleMode("LAND")
    #     while vehicle.mode!='LAND':
    #         print('WAITING FOR DRONE TO ENTER LAND MODE')
    #         time.sleep(1)

    print('prepare to start TRY block')
    try:
        print('started TRY block')
        if ids is not None and ids[0] == id_to_find:
            print('I SAW IT')
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])

            y_sum = 0
            x_sum = 0

            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]

            x_avg = x_sum*.25
            y_avg = y_sum*.25

            x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
            y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)

            ########### PID CONTROL ##########

            pid_x = PID(0.25, 0.5, 0.01,setpoint=x_ang)
            pid_y = PID(0.25, 0.5, 0.01,setpoint=y_ang)

            pid_x.setpoint = 0
            pid_y.setpoint = 0

            x_ang_control = pid_x(x_ang)
            y_ang_control = pid_y(y_ang)

            px, ix, dx = pid_x.components
            py, iy, dy = pid_y.components

            #send_land_message(x_ang,y_ang)
            #send_land_message(-x_ang_control,-y_ang_control)
            print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            print("MARKER POSITION: x=" +x+" y= "+y+" z="+z)
            found_count=found_count+1
            print("")
            print(x_ang,y_ang)
            print(-x_ang_control,-y_ang_control)
            print("########################################################")
            return None 
        else:
            notfound_count=notfound_count+1
            print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            return None
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
        return None
