#include "ardupilot_flightmode.h"
using namespace Ardupilot;

ArdupilotFlightMode::ArdupilotFlightMode()
{
    availableFM = &arducopterFM;
    this->vehicleType = MAV_TYPE_GENERIC;
    this->flightMode = ACFM_UNKNOWN;
}

//void ArdupilotFlightMode::receivedMessage(const int &msgType, const T &data){
//    switch (msgType) {
//    case HEARTBEATUPDATE:
//        STRUCT_HEARTBEATUPDATE tmp = dynamic_cast<STRUCT_HEARTBEATUPDATE>(data);
//        break;
//    default:
//        break;
//    }
//}

void ArdupilotFlightMode::setVehicleType(int vehicleType){
    switch (vehicleType) {
    case MAV_TYPE_FIXED_WING:
        this->vehicleType = vehicleType;
        this->flightMode = APFM_UNKNOWN;
        availableFM = &arduplaneFM;
        break;
    case MAV_TYPE_TRICOPTER:
    case MAV_TYPE_QUADROTOR:
    case MAV_TYPE_HEXAROTOR:
    case MAV_TYPE_OCTOROTOR:
        this->vehicleType = vehicleType;
        this->flightMode = ACFM_UNKNOWN;
        availableFM = &arducopterFM;
        break;
    default:
        break;
    }
}
