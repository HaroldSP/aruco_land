#!/usr/bin/env python
# -*- coding: utf-8 -*-

#THIS IS A FLY SCRIPT OF PL, READY TO BE TESTED IN FIELD

##########DEPENDENCIES################
#from functions import *              # my custom imports
#import multicam as M               # my custom imports
from threading import Thread, Lock



import argparse
import cv2
import cv2.aruco as aruco
import datetime
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

#######these are for passing args to a py script########################

id_to_find = 72 ##Aruco
marker_size = 20 #cm

takeoff_height = 1
velocity = .25

script_mode = 1  # 1 for arm and takeoff, 2 for manual LOITER to GUIDED land
ready_to_land = 0  # 1 to trigger landing

# If True, arming from RC controller, If False, arming from this script.
manualArm = False

##Counters and script triggers
found_count = 0
notfound_count = 0

first_run = 0  # Used to set initial time of function to determine FPS
start_time = 0
end_time = 0

#####Camera#######
horizontal_res = 640
vertical_res = 480

##################
horizontal_fov = 62.2 * (math.pi / 180)  # Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)  # Pi cam V1: 41.41 V2: 48.8
##################

##################
calib_path = "/home/pi/video2calibration/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
##################

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()
##################

#starting videostream here, before the loop
vs = WebcamVideoStream()
vs.stop()
vs.start()

#########LANDER FUNCTION#################
class WebcamVideoStream:
    def __init__(self, src=0, width=640, height=480):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        (self.grabbed, self.frame) = self.stream.read()
        self.started = False
        self.read_lock = Lock()

    def start(self):
        if self.started:
            print "already started!!"
            return None
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        while self.started:
            (grabbed, frame) = self.stream.read()
            self.read_lock.acquire()
            self.grabbed, self.frame = grabbed, frame
            self.read_lock.release()

    def read(self):
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()
        return frame

    def stop(self):
        self.started = False
        self.thread.join()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stream.release()

def connectMyCopter():

    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    if not connection_string:
        connection_string='127.0.0.1:14550'
    vehicle = connect(connection_string,wait_ready=True)
    return vehicle

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable!=True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode!='GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")

    if manualArm==False:
        vehicle.armed = True
        while vehicle.armed==False:
            print("Waiting for vehicle to become armed.")
            time.sleep(1)
    else:
        if vehicle.armed == False:
            print("Exiting script. manualArm set to True but vehicle not armed.")
            print("Set manualArm to True if desiring script to arm the drone.")
            return None
    print("Look out! Props are spinning!!")

    vehicle.simple_takeoff(targetHeight) ##meters

    while True:
        print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")
    return None

def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0,)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def lander():
    global first_run, notfound_count, found_count, marker_size, start_time, vs
    global p, i, d, pid, marker_size

    if first_run == 0:
        print("First run of lander!!")
        first_run = 1
        start_time = time.time()
    frame = vs.read()
    frame = cv2.resize(frame, (horizontal_res, vertical_res))
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
    ids = ''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

    print("DRONE IS IN THE LAND MODE - checking in loop")
    if vehicle.mode!='LAND':
        vehicle.mode=VehicleMode("LAND")
        while vehicle.mode!='LAND':
            print('WAITING FOR DRONE TO ENTER LAND MODE')
            time.sleep(1)
    try:
        print('started TRY block')
        if ids is not None and ids[0] == id_to_find:
            print('I SAW IT')
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=cameraMatrix, distCoeffs=cameraDistortion)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])

            y_sum = 0
            x_sum = 0

            x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
            y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

            x_avg = x_sum*.25
            y_avg = y_sum*.25

            x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
            y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)

            ########### PID CONTROL ##########

            pid_x = PID(0.25, 0.5, 0.01, setpoint=x_ang)
            pid_y = PID(0.25, 0.5, 0.01, setpoint=y_ang)

            pid_x.setpoint = 0
            pid_y.setpoint = 0

            x_ang_control = pid_x(x_ang)
            y_ang_control = pid_y(y_ang)

            px, ix, dx = pid_x.components
            py, iy, dy = pid_y.components

            #PL COMTROL STUFF WITHOUT PID
            #F.send_land_message(x_ang,y_ang)

            #THIS IS THE MOST IMPORTANT THING HERE - PL WITH PID CONTROL
            send_land_message(-x_ang_control,-y_ang_control)
            ############################################################

            print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            print("FOUND COUNT: "+str(found_count) + " NOTFOUND COUNT: "+str(notfound_count))
            print("MARKER POSITION: x=" + x+" y= "+y+" z="+z)
            found_count = found_count+1
            print("")
            print(x_ang, y_ang)
            print(-x_ang_control, -y_ang_control)
            print("########################################################")
            return None
        else:
            notfound_count = notfound_count+1
            print("FOUND COUNT: "+str(found_count) + " NOTFOUND COUNT: "+str(notfound_count))
            return None
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
        return None

if __name__ == '__main__':

    try:
        ################Connecting to the drone##############

        vehicle = connectMyCopter()
        while vehicle != vehicle:
            print("Waiting for vehicle to connect to mavlink.")
            time.sleep(1)
        print("Successfully connected to vehicle")
        ######################################################

        #SETUP PARAMETERS TO ENABLE PRECISION LANDING
        #
        vehicle.parameters['PLND_ENABLED'] = 1
        vehicle.parameters['PLND_TYPE'] = 1 ##1 for companion computer
        vehicle.parameters['PLND_EST_TYPE'] = 0 ##0 for raw sensor, 1 for kalman filter pos estimation
        vehicle.parameters['LAND_SPEED'] = 15 ##Descent speed of 15cm/s

        #######################################################

        if script_mode ==1:
            arm_and_takeoff(takeoff_height)
            ############2#######################
            print("FLYING UP!!", takeoff_height)
            ####################################
            today = datetime.datetime.today()
            print(today.strftime("%Y-%m-%d-%H.%M.%S"))  # 2017-04-05-00.18.00 format
            #F.send_local_ned_velocity(velocity,velocity,0) ##Offset drone from target
            time.sleep(1)
            ready_to_land=1

        elif script_mode==2:
            while vehicle.mode!='GUIDED':
                time.sleep(1)
                print("Waiting for manual change from mode "+str(vehicle.mode)+" to GUIDED")
            ready_to_land=1

        if ready_to_land==1:

            ####################################
            print("ENTERING ready_to_land SECTION in MAIN")
            ####################################

            while vehicle.armed==True:
                lander()
                #time.sleep(1)
                print("ARMED:", vehicle.armed) #might need to comment out later

            print('LANDED')
            vs.stop()
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

    except Exception as e:

        vs.stop()
        print("THIS IS THE END, LANDED or whatever")
        print("THIS WAS BY AN EXCEPTION, closing VS")
        print('This is an Error: '+str(e))
        # end_time = time.time()
        # total_time=end_time-start_time
        # #total_time=abs(int(total_time))

        # total_count=found_count+notfound_count
        # freq_lander=abs(float(total_count/total_time))
        # print("Total iterations: "+str(total_count))
        # print("Total seconds: "+str(total_time))
        # print("------------------")
        # print("lander function had frequency of: "+str(freq_lander))
        # print("------------------")
        # print("Vehicle has landed")
        # print("------------------")
        pass
