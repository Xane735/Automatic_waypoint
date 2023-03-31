import csv
import os
import math
from pymavlink import mavutil
from pymavlink import mavwp
import numpy as np
from math import sqrt
import re
import time

# Set up the connection to the autopilot
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the heartbeat message to ensure the autopilot is connected
master.wait_heartbeat()

# Converting .waypoint file to .csv file
"""waypoint_file_path = "D:\P\MSRIT\EDITHA\TEST WAYPOINTS\random1.waypoints"
csv_file_path = "D:\P\MSRIT\EDITHA\TEST WAYPOINTS\random1.waypoints"

with open(waypoint_file_path, "r") as waypoint_file:
    with open(csv_file_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["waypoint_number", "is_home_?","confirmation", "dont_know_1","param_1", "param_2", "param_3", "param_4", "latitude", "longitude", "altitude","dont_know_2"])

        for line in waypoint_file:
            values = line.strip().split(",")
            latitude = values[8]
            longitude = values[9]
            altitude = values[10]
            writer.writerow([latitude, longitude, altitude])

with open(csv_file_path, "r") as csv_file:
    reader = csv.reader(csv_file)

    for row in reader:
        print(row)"""
# Load the waypoint file
waypoint_file = open('D:\P\MSRIT\EDITHA\TEST WAYPOINTS\random1.waypoints')
waypoints =[]
for line in waypoint_file:
    # Parse the line and create a waypoint object
    lat, lon, alt = line.split(',')
    waypoint = mavutil.mavlink.Mavlink_mission_item_messgae(0, 0, 0, 16, 0, 0, 0, float(lat), float(lon), float(alt), 0, 0, 0,0)
    waypoint_file.close()
# Clear existing mission
master.mav.mission_clear_all_send()

# Send the new mission items to the autopilot
for i in range(len(waypoints)):
    master.mav.mission_item_send(
        0, 0, i, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
        waypoints[i].x, waypoints[i].y, waypoints[i].z)

# Arm the vehicle and start the mission
master.mav.command_long_send(
    master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)

master.mav.command_long_send(
    master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_MISSION_START,
    0, 0, 0, 0, 0, 0, 0, 0)

# Wait for the mission to complete
while True:
    msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
    if not msg:
        continue
    if msg.seq == len(waypoints) - 1:
        print("Mission complete!")
        break

# Disarm the vehicle
master.mav.command_long_send(
    master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0)

# Close the connection to the autopilot
master.close()
