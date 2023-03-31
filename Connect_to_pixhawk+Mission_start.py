import time
from pymavlink import mavutil
from pymavlink import mavwp
def mission_start():

# to Start the mission
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_MISSION_START,0, 0, 0, 0, 0)
from pymavlink import mavutil

# to start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Waiting for first heartbeat message 
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Connecting to the autopilot
master = mavutil.mavlink_connection('udpout:localhost:14550')

# Connecting to Pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# Loading Waypoint file 
waypoints = []
with open('waypoints.txt') as f:
    for line in f:
        lat, lon, alt = line.strip().split(',')
        waypoints.append((float(lat), float(lon), float(alt)))

# Writing waypoint file to Pixhawk
for i, waypoint in enumerate(waypoints):
    msg = mavutil.mavlink.MAVLink_mission_item_message(0, 0, i, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint[0], waypoint[1], waypoint[2])
    master.mav.send(msg)

# Load a mission file
"""mission_file = "path/to/mission/file.waypoints"
with open(mission_file) as f:
    mission_data = f.read().replace('\n', '')
master.waypoint_clear_all_send()
master.waypoint_count_send(len(mission_data))
for i, waypoint in enumerate(mission_data):
    master.waypoint_send(waypoint.frame, waypoint.command, i, waypoint.current, waypoint.autocontinue,
                         waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4, waypoint.x,
                         waypoint.y, waypoint.z)"""

# Arm
master.arducopter_arm()

# Wait for arming to complete
while not master.motors_armed():
    print("Waiting for arming...")
    time.sleep(1)

# Starting the mission
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0)

# Waiting for the mission to end
while True:
    msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
    if not msg:
        continue
    if msg.seq == len(waypoints) - 1:
        print("Mission complete!")
        break

# Disarm 
master.mav.command_long_send(
    master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0)

# Ending connection
master.close()
