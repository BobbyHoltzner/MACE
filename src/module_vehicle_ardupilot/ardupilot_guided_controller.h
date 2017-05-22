#ifndef ARDUPILOT_GUIDED_CONTROLLER_H
#define ARDUPILOT_GUIDED_CONTROLLER_H

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

#include "ardupilot_general_controller.h"
#include "ardupilot_mission_state.h"

class Ardupilot_GuidedController : public Ardupilot_GeneralController
{
public:

    MissionItem::MissionList getDummyMissionList() const{
        return m_CurrentMission;
    }

    Ardupilot_GuidedController(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan, callbackFunction callback);

    ~Ardupilot_GuidedController() {
        std::cout << "Destructor on guidance controller" << std::endl;
    }

    void initializeMissionSequence();
    void updatedMission(const MissionItem::MissionList &updatedMission);

    double distanceToTarget();
    void generateControl(const Data::ControllerState &currentState);
    void updateCommandACK(const mavlink_command_ack_t &cmdACK);
    void run();

private:
    //FLAGS for the thread:
    bool executionState;
    bool missionUpdated;

    MissionItem::MissionList m_CurrentMission;
    CommandItem::SpatialHome m_VehicleHome;

};


#endif // ARDUPILOT_GUIDED_CONTROLLER_H
