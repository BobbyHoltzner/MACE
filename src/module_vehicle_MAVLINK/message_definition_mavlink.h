#ifndef MESSAGE_DEFINITION_MAVLINK_H
#define MESSAGE_DEFINITION_MAVLINK_H

#include <string>
#include "mavlink.h"

#include "mace_core/vehicle_message.h"
#include "mace_core/vehicle_object.h"

struct HEARTBEATData: public AbstractVehicleMessage{
    HEARTBEATData(const int &vehicleID)
    {
        m_VehicleID = vehicleID;
    }
    void updateFromMAVLINK(const mavlink_heartbeat_t &dataObject){
        custom_mode = dataObject.custom_mode;
        type = parseVehicleType(dataObject.type);
        autopilot = dataObject.autopilot;
        base_mode = dataObject.base_mode;
        system_status = dataObject.system_status;
        mavlink_version = dataObject.mavlink_version;
    }

    virtual std::string getDescription() const{
        std::string rtnString = "This message contains HEARTBEAT data";
        return rtnString;
    }
    virtual std::string getMessageType()const{
        return "MAVLINK_MSG_HEARTBEAT";
    }

    VehicleTypeENUM parseVehicleType(const uint8_t &type){
        int returnValue = 0;
        switch (type) {
        case MAV_TYPE_GENERIC:
            returnValue = VT_GENERIC;
            break;
        case MAV_TYPE_TRICOPTER:
            returnValue = VT_TRICOPTER;
            break;
        case MAV_TYPE_QUADROTOR:
            returnValue = VT_QUADROTOR;
            break;
        case MAV_TYPE_HEXAROTOR:
            returnValue = VT_HEXACOPTER;
            break;
        case MAV_TYPE_OCTOROTOR:
            returnValue = VT_OCTOCOPTER;
            break;
        case MAV_TYPE_HELICOPTER:
            returnValue = VT_HELICOPTER;
            break;
        case MAV_TYPE_FIXED_WING:
            returnValue = VT_FIXED_WING;
            break;
        default:
            returnValue = 0;
            break;
        }
    }

    uint32_t custom_mode; /*< A bitfield for use for autopilot-specific flags.*/
    VehicleTypeENUM type; /*< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)*/
    uint8_t autopilot; /*< Autopilot type / class. defined in MAV_AUTOPILOT ENUM*/
    uint8_t base_mode; /*< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h*/
    uint8_t system_status; /*< System status flag, see MAV_STATE ENUM*/
    uint8_t mavlink_version; /*< MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/
};

struct ATTITUDEData: public AbstractVehicleMessage{
    ATTITUDEData(const int &vehicleID)
    {
        m_VehicleID = vehicleID;

    }
    void updateFromMAVLINK(const mavlink_attitude_t &dataObject){
        roll = dataObject.roll;
        rollspeed = dataObject.rollspeed;
        pitch = dataObject.pitch;
        pitchspeed = dataObject.pitchspeed;
        yaw = dataObject.yaw;
        yawspeed = dataObject.yawspeed;
    }

    virtual std::string getDescription() const{
        std::string rtnString = "This message contains ATTITUDE data";
        return rtnString;
    }
    virtual std::string getMessageType() const{
        return "MAVLINK_MSG_ATTITUDE";
    }

    double roll; /*< Roll angle (rad, -pi..+pi)*/
    double rollspeed; /*< Roll angular speed (rad/s)*/
    double pitch; /*< Pitch angle (rad, -pi..+pi)*/
    double pitchspeed; /*< Pitch angular speed (rad/s)*/
    double yaw; /*< Yaw angle (rad, -pi..+pi)*/
    double yawspeed; /*< Yaw angular speed (rad/s)*/

};

struct VFRData: public AbstractVehicleMessage{
    VFRData(const int &vehicleID)
    {
        m_VehicleID = vehicleID;
    }

    virtual std::string getDescription() const{
        std::string rtnString = "This message contains VFR data";
        return rtnString;
    }
    virtual std::string getMessageType() const{
        return "MAVLINK_MSG_VFR";
    }
    double airspeedData;
};

struct GPSRAWData: public AbstractVehicleMessage{
    GPSRAWData(const int &vehicleID)
    {
        m_VehicleID = vehicleID;

    }
    void updateFromMAVLINK(const mavlink_gps_raw_int_t &dataObject){
        this->time_usec = dataObject.time_usec;
        this->latitude = dataObject.lat / pow(10,7);
        this->longitude = dataObject.lon / pow(10,7);
        this->altitude = dataObject.alt / 1000.0;
        this->eph = dataObject.eph;
        this->epv = dataObject.epv;
        this->vel = dataObject.vel;
        this->cog = dataObject.cog / 100.0;
        this->fix_type = dataObject.fix_type;
        this->satellites_visible = dataObject.satellites_visible;
    }

    virtual std::string getDescription() const{
        std::string rtnString = "This message contains ATTITUDE data";
        return rtnString;
    }
    virtual std::string getMessageType() const{
        return "MAVLINK_MSG_GPSRAW";
    }

    uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
    double latitude; /*< Latitude (WGS84), in degrees * 1E7*/
    double longitude; /*< Longitude (WGS84), in degrees * 1E7*/
    double altitude; /*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.*/
    uint16_t eph; /*< GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
    uint16_t epv; /*< GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
    uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
    uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
    uint8_t fix_type; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
    uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
};

#endif // MESSAGE_DEFINITION_MAVLINK_H
