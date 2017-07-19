#ifndef ARDUPILOT_GENERAL_CONTROLLER_H
#define ARDUPILOT_GENERAL_CONTROLLER_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>
#include <mavlink.h>

#include "data/controller_state.h"

#include "data_interface_MAVLINK/vehicle_object_mavlink.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_ardupilot/components.h"
#include "data_vehicle_ardupilot/vehicle_object_ardupilot.h"

#include "comms/comms_marshaler.h"

#include "data/threadmanager.h"
#include "ardupilot_mission_state.h"

#include <list>

class Ardupilot_GeneralController : public Thread
{
public:
    enum controllerTypes
    {
        CONTROLLER_GUIDED,
        CONTROLLER_TAKEOFF
    };

public:
    Ardupilot_GeneralController(std::shared_ptr<DataInterface_MAVLINK::VehicleObject_MAVLINK> vehicleData);

    ~Ardupilot_GeneralController() {
        std::cout << "Destructor on general controller" << std::endl;
        mToExit = true;
    }

    void terminateObject();

    virtual void updateCommandACK(const mavlink_command_ack_t &cmdACK) = 0;


protected:
    controllerTypes controllerType;

protected:
    std::shared_ptr<DataInterface_MAVLINK::VehicleObject_MAVLINK> vehicleDataObject;

protected:
    //FLAGS for the thread:
    bool mToExit;

    //Methods for determining state of the vehicle
    ArdupilotMissionState vehicleMissionState;

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
