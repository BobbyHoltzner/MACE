#include "ardupilot_flightmode.h"
#include <iostream>

using namespace Ardupilot;

ArdupilotFlightMode::ArdupilotFlightMode()
{
    availableFM = &arducopterFM;
    this->vehicleType = MAV_TYPE_GENERIC;
    this->flightMode = ACFM_UNKNOWN;
}

void ArdupilotFlightMode::getCurrentVehicleMode(int vehicleMode)
{
    vehicleMode = flightMode;
}

void ArdupilotFlightMode::getCurrentVehicleMode(std::string &vehicleMode)
{
    vehicleMode = availableFM->at(flightMode);
}

void ArdupilotFlightMode::setFlightMode(int flightMode)
{
    this->flightMode = flightMode;
}

void ArdupilotFlightMode::setVehicleType(int vehicleType){
    this->flightMode = APFM_UNKNOWN;

    switch (vehicleType) {
    case MAV_TYPE_FIXED_WING:
        this->vehicleType = vehicleType;
        availableFM = &arduplaneFM;
        break;
    case MAV_TYPE_TRICOPTER:
    case MAV_TYPE_QUADROTOR:
    case MAV_TYPE_HEXAROTOR:
    case MAV_TYPE_OCTOROTOR:
        this->vehicleType = vehicleType;
        availableFM = &arducopterFM;
        break;
    default:
        break;
    }
}
