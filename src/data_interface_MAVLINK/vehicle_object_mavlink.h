#ifndef VEHICLE_OBJECT_MAVLINK_H
#define VEHICLE_OBJECT_MAVLINK_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>

#include "comms/comms_marshaler.h"

#include "command_interface_mavlink.h"

namespace DataInterface_MAVLINK{

class VehicleObject_MAVLINK
{
public:
    VehicleObject_MAVLINK(const int &vehicleID, const int &transmittingID);

    void updateCommsInfo();

    void transmitMessage(const mavlink_message_t &msg);

    void parseMessage(const mavlink_message_t &msg);

    void transmitCommandLong(const mavlink_command_long_t cmd);

//The following establish the necessary callback routines
public:
    static void staticCallbackCMDLongFunction(void *p, mavlink_command_long_t cmd)
    {
        ((VehicleObject_MAVLINK *)p)->transmitCommandLong(cmd);
    }

    //The following are organizational methods to compartmentalize funcitonality
public:
    CommandInterface_MAVLINK *command;

    //The following are members describing important details of the vehicle object
private:
    int systemID;
    int commandID;

    //The following are members enabling communications with the vehicle object
private:
    Comms::CommsMarshaler *m_LinkMarshaler;
    std::string m_LinkName;
    uint8_t m_LinkChan;
};

} //end of namespace DataInterface_MAVLINK
#endif // VEHICLE_OBJECT_MAVLINK_H
