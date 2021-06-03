#!/usr/bin/env python
# -*- coding: utf-8 -*-

##########DEPENDENCIES################
import argparse
import time

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
from pymavlink import mavutil

#manualArm=False ##If True, arming from RC controller, If False, arming from this script.

#########FUNCTIONS#################

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
