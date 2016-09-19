import sys, struct, time, os, threading, math
from pymavlink.dialects.v20 import common as mavlink
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from argparse import ArgumentParser
from threading import Thread

missionLength = 0
currentMissionIndex = 0
latArray = []
lngArray = []
altArray = []

def vehicle_LocationCallback(self, attr_name, value):
    print("The vehicle location is %f %f %f: " %(value.lat, value.lon, value.alt))


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def goto(latitude, longitude, altitude, radius):
    print "I am in the go to function"
    targetLocation = LocationGlobal(latitude, longitude, altitude)
    goto_position_target_global_int(targetLocation)

    while vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
        # print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print "Distance to target: ", remainingDistance
        #goto_position_target_global_int(targetLocation)
        if remainingDistance <= radius:  # Just below target, in case of undershoot.
            print "Reached target"
            break;
        time.sleep(1)


def goto_position_target_global_int(targetLocation):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        targetLocation.lat * 1e7,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        targetLocation.lon * 1e7,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        targetLocation.alt,
        # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0,  # X velocity in NED frame in m/s
        0,  # Y velocity in NED frame in m/s
        0,  # Z velocity in NED frame in m/s
        0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def arm_and_takeoff(targetAltitude):
    while not vehicle.is_armable:
        print("The vehicle cannot arm in this state")
        return
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for the vehicle to arm")
        time.sleep(0.5)

    print "I have told the vehicle to takeoff to an altitude of %s" %targetAltitude
    vehicle.simple_takeoff(targetAltitude)
#Even though it was convienent to know when the vehicle was done climbing...it was impractical as it blocked any
#other events from happening...and launching on own thread doesnt seem practical
#    while True:
#        if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
#            break
#        time.sleep(0.5)
#    print "The vehicle is done climbing"

parser = ArgumentParser(description=__doc__)
parser.add_argument("--baudrate", type=int,
                  help="master port baud rate", default=57600)
parser.add_argument("--device", help="serial device", default="com5")
parser.add_argument("--rate", default=4, type=int, help="requested stream rate")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                  default=255, help='MAVLink source system for this GCS')
parser.add_argument("--showmessages", action='store_true',
                  help="show incoming messages", default=True)
args = parser.parse_args()

def heartbeatTimer(m, interval):
    mavlinkMSG = mavlink.MAVLink(m)
    while True:
        time.sleep(interval)
        mavlinkMSG.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, mavlink.MAV_STATE_ACTIVE)

        home = LocationGlobal(vehicle.home_location.lat, vehicle.home_location.lon, vehicle.home_location.alt)
        mavlinkMSG.mission_item_send(0, 0, 0, 0, 0, 0, 0, vehicle.location.global_frame.lat,
                                     vehicle.location.global_frame.lon, vehicle.location.global_frame.alt, vehicle.heading, home.lat,
                                     home.lon, home.alt)

        #msg = m.recv_match(type="COMMAND_ACK", blocking=False)
        #if msg:
        #    break


def handleMissionMessage(m):
    mavlinkMSG = mavlink.MAVLink(m)
    global latArray
    global lngArray
    global altArray

    '''show incoming mavlink messages'''
    while True:
        msg = m.recv_match(blocking=True)
        if not msg:
            continue

        msgID = msg.get_msgId()
        if msgID == mavlink.MAVLINK_MSG_ID_MISSION_ITEM:
            if(msg.command == mavlink.MAV_CMD_NAV_WAYPOINT):
                print"I saw a new mission object for index %s" % msg.seq
                index = msg.seq + 1
                latArray.insert(index, msg.x)
                lngArray.insert(index, msg.y)
                altArray.insert(index, msg.z)
                currentMissionIndex = currentMissionIndex + 1
                if currentMissionIndex < missionLength:
                    mavlinkMSG.mission_request_send(m.target_system, m.target_component, currentMissionIndex)
                else:
                    print "I am done with requesting mission items"
            elif(msg.command == mavlink.MAV_CMD_NAV_TAKEOFF):
                print "I am going to perform a takeoff"
                arm_and_takeoff(20)
            elif(msg.command == mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH):
                print "I am going to perform a RTL"
                vehicle.mode = VehicleMode("RTL")
            elif(msg.command == mavlink.MAV_CMD_NAV_LAND):
                print "I am going to perform a land"
                vehicle.mode = VehicleMode("LAND")
            elif(msg.command == mavlink.MAV_CMD_DO_GUIDED_MASTER):
                print "I have been allowed to perform my guided mode"
                thread = Thread(target = performMissionFunciton)
                thread.start()
        elif msgID == mavlink.MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            print "I saw a clear all command"
            latArray = []
            lngArray = []
            altArray = []
            #probably want to halt motion and wait for further commands as well
        elif msgID == mavlink.MAVLINK_MSG_ID_MISSION_COUNT:
            print("I saw a mission count %s" %msg.count)
            missionLength = msg.count
            currentMissionIndex = 0
            mavlinkMSG.mission_request_send(m.target_system, m.target_component, currentMissionIndex)
        elif  msgID == mavlink.MAVLINK_MSG_ID_BAD_DATA:
            if(mavutil.all_printable(msg.data)):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else:
            print(msg)


def performMissionFunciton():
    for i in range(0, len(latArray), 1):
        goto(latArray[i], lngArray[i], altArray[i], 5)

if __name__ == '__main__':
    vehicle = connect("udp:127.0.0.1:14551", wait_ready=True)
# Get Vehicle Home location - will be `None` until first set by autopilot
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print " Waiting for home location ..."

    # We have a home location, so print it!
    print "\n Home location: %s" % vehicle.home_location

    # create a mavlink serial instance that connects to the ground station
    # mavutil.mavlink_connection("com5", 57600)
    master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

    mavlinkMSG = mavlink.MAVLink(master)
    heartbeatThread = Thread(target = heartbeatTimer, args = (master,1))
    heartbeatThread.start()
    #heartbeatTimer(master, 2)
#    home = LocationGlobal(vehicle.home_location.lat, vehicle.home_location.lon, vehicle.home_location.alt)
#    mavlinkMSG.mission_item_send(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, home.lat, home.lon, home.alt)

    handleMissionMessage(master)

    #handleFlightMessages(master)