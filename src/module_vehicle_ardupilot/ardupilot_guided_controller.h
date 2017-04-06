#ifndef ARDUPILOT_GUIDED_CONTROLLER_H
#define ARDUPILOT_GUIDED_CONTROLLER_H

#include <thread>
#include <string>
#include <iostream>

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_mission_item/mission_item_components.h"

class Ardupilot_GuidedController
{
public:
    Ardupilot_GuidedController();

public:
    void runGuidanceRoutine();
    void flagGuidance(const bool &execute);
    void updateVehiclePosition(const DataState::StateGlobalPosition &currentPosition);
    void updateVehicleMode(const std::string &vehicleMode);

private:
    bool execute;
    std::string vehicleMode;
    DataState::StateGlobalPosition currentPosition;

};

#endif // ARDUPILOT_GUIDED_CONTROLLER_H
