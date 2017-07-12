#ifndef VEHICLE_OBJECT_MAVLINK_H
#define VEHICLE_OBJECT_MAVLINK_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>

#include "comms/comms_marshaler.h"

#include "command_interface_mavlink.h"

#include "mission_controller_mavlink.h"
#include "mission_data_mavlink.h"
#include "state_data_mavlink.h"

namespace DataInterface_MAVLINK{

class VehicleObject_MAVLINK : public MissionController_Interface
{
public:
    VehicleObject_MAVLINK(const int &vehicleID, const int &transmittingID);

    ~VehicleObject_MAVLINK();

    void updateCommsInfo(Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan);

    void transmitMessage(const mavlink_message_t &msg);

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> ParseForVehicleData(const mavlink_message_t* message);

    void parseMessage(const mavlink_message_t &msg);

    void transmitCommandLong(const mavlink_command_long_t &cmd);

    void receiveCommand(const DataState::StateGlobalPosition &pos);

//The following establish the necessary callback routines

    //The following are as required from the mission controller interface
public:
    void cbiMissionController_TransmitMissionCount(const mavlink_mission_count_t &count);
    void cbiMissionController_TransmitMissionItem(const mavlink_mission_item_t &item);

    void cbiMissionController_TransmitMissionReqList(const mavlink_mission_request_list_t &request);
    void cbiMissionController_TransmitMissionReq(const mavlink_mission_request_t &requestItem);

    void cbiMissionController_ReceviedHome(const CommandItem::SpatialHome &home);
    void cbiMissionController_ReceivedMission(const MissionItem::MissionList &mission);

public:
    static void staticCallbackCMDLongFunction(void *p, mavlink_command_long_t &cmd)
    {
        ((VehicleObject_MAVLINK *)p)->transmitCommandLong(cmd);
    }

    static void staticCallbackState(void *p, DataState::StateGlobalPosition &pos)
    {
        ((VehicleObject_MAVLINK *)p)->receiveCommand(pos);
    }

    static void staticCallbackTransmitMissionMSG(void *p, mavlink_message_t &msg)
    {
        ((VehicleObject_MAVLINK *)p)->transmitMessage(msg);
    }

    static void staticCallbackTransmitMSG(void *p, mavlink_message_t &msg)
    {
        ((VehicleObject_MAVLINK *)p)->transmitMessage(msg);
    }

    //The following are basic controllers for the MAVLINK vehicle object
public:
    MissionController_MAVLINK *missionController;

    //The following are organizational methods to compartmentalize funcitonality
public:
    CommandInterface_MAVLINK *command;
    MissionData_MAVLINK *mission;
    StateData_MAVLINK *state;

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
