#ifndef ARDUPILOT_TAKEOFF_CONTROLLER_H
#define ARDUPILOT_TAKEOFF_CONTROLLER_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>
#include <mavlink.h>

#include "data/controller_state.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_ardupilot/components.h"
#include "data_vehicle_ardupilot/vehicle_object_ardupilot.h"

#include "comms/comms_marshaler.h"

#include "ardupilot_general_controller.h"
#include "ardupilot_mission_state.h"

class Ardupilot_TakeoffController : public Ardupilot_GeneralController
{
public:

    Ardupilot_TakeoffController(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan);

    ~Ardupilot_TakeoffController();

    void initializeTakeoffSequence(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &takeoff);
    void updatedFlightMode(const DataARDUPILOT::VehicleFlightMode &flightMode);

    double distanceToTarget();
    void controlSequence();
    void generateControl(const Data::ControllerState &currentState);
    void updateCommandACK(const mavlink_command_ack_t &cmdACK);

    void run();

private:
    enum stateLogic{
        DISARMED,
        ARMED_WRONG_MODE,
        ARMED_RIGHT_MODE,
        ALTITUDE_TRANSITION,
        HORIZONTAL_TRANSITION
    };

    stateLogic currentStateLogic;

private:
    MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> missionItem_Takeoff;
};

#endif // ARDUPILOT_TAKEOFF_CONTROLLER_H
