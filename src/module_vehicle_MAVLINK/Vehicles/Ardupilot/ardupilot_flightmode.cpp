#include "ardupilot_flightmode.h"
#include <iostream>

using namespace Ardupilot;

ArdupilotFlightMode::ArdupilotFlightMode()
{
    availableFM = arducopterFM;
    this->vehicleType = MAV_TYPE_GENERIC;
    this->flightMode = ACFM_UNKNOWN;
}

void ArdupilotFlightMode::getCurrentVehicleMode(int vehicleMode)
{
    vehicleMode = flightMode;
}

void ArdupilotFlightMode::getCurrentVehicleMode(std::string &vehicleMode)
{
    vehicleMode = availableFM.at(flightMode);
}

bool ArdupilotFlightMode::getVehicleModeID(const std::string &vehicleModeString, int vehicleModeID){

    std::map<int,std::string>::iterator it;
    for (it=availableFM.begin(); it != availableFM.end(); it++)
    {
        if(it->second == vehicleModeString)
        {
            std::cout<<"They had matched"<<std::endl;
            vehicleModeID = it->first;
            return true;
        }
    }
    return false;
}

void ArdupilotFlightMode::setFlightMode(int flightMode)
{
    this->flightMode = flightMode;
}

void ArdupilotFlightMode::setVehicleType(int vehicleType){
    if(this->vehicleType != vehicleType)
    {
        switch (vehicleType) {
        case MAV_TYPE_FIXED_WING:
        {
            this->vehicleType = vehicleType;
            availableFM = arduplaneFM;
            break;
        }
        case MAV_TYPE_TRICOPTER:
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        {
            this->vehicleType = vehicleType;
            availableFM = arducopterFM;
            break;
        }
        default:
            break;
        }
    }
}
