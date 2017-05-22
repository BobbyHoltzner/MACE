#ifndef ARDUPILOT_GENERAL_CONTROLLER_H
#define ARDUPILOT_GENERAL_CONTROLLER_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>
#include <mavlink.h>

#include "data/controller_state.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_ardupilot/components.h"
#include "data_vehicle_ardupilot/vehicle_object_ardupilot.h"

#include "comms/comms_marshaler.h"

#include "Data/threadmanager.h"
#include "ardupilot_mission_state.h"

#include <functional>
#include <list>

using callbackFunction = std::function<void(std::string)>;

class Ardupilot_GeneralController : public Thread
{
public:
    enum controllerTypes
    {
        CONTROLLER_GUIDED,
        CONTROLLER_TAKEOFF
    };

public:
    Ardupilot_GeneralController(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan, callbackFunction callback);

    ~Ardupilot_GeneralController() {
        std::cout << "Destructor on general controller" << std::endl;
        mToExit = true;
    }

    void terminateObject();

    /*
    virtual void updateFlightMode(const DataARDUPILOT::VehicleFlightMode &flightMode, const bool &updateFlag  = true);
    virtual void updatedHomePostion(const MissionItem::SpatialHome &homePosition, const bool &updateFlag  = true);
    virtual void updateAttitudeTopic(const DataState::StateAttitude &attitude, const bool &updateFlag  = true);
    virtual void updateGlobalPositionTopic(const DataState::StateGlobalPosition &globalPosition, const bool &updateFlag  = true);
    */
    virtual void updateCommandACK(const mavlink_command_ack_t &cmdACK) = 0;


protected:
    controllerTypes controllerType;
    //The following are communications objects
protected:
    Comms::CommsMarshaler *m_LinkMarshaler;
    std::string m_LinkName;
    uint8_t m_LinkChan;

protected:
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleDataObject;

protected:
    //FLAGS for the thread:
    bool mToExit;

    //Methods for determining state of the vehicle
    ArdupilotMissionState vehicleMissionState;

    /*
    DataARDUPILOT::VehicleFlightMode currentVehicleMode;
    MissionItem::SpatialHome currentHome;
    DataState::StateGlobalPosition currentPosition;
    DataState::StateAttitude currentAttitude;
    */

    // Callback:
    std::string m_dataAvailable;
    callbackFunction m_callback;

protected:


    std::list<std::function<void()>> m_LambdasToRun;

    void RunPendingTasks() {
        while(m_LambdasToRun.size() > 0) {
            auto lambda = m_LambdasToRun.front();
            m_LambdasToRun.pop_front();
            lambda();
        }
    }

};

#endif // ARDUPILOT_GENERAL_CONTROLLER_H
