#include "ardupilot_general_controller.h"

Ardupilot_GeneralController::Ardupilot_GeneralController(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan):
    vehicleDataObject(vehicleData),
    m_LinkMarshaler(commsMarshaler),m_LinkName(linkName),m_LinkChan(linkChan), mToExit(false),
    attitudeUpdated(false),positionUpdated(false),modeUpdated(false)
{
    std::cout << "Constructor on general controller" << std::endl;
}

/*
void Ardupilot_GeneralController::updateAttitudeTopic(const DataState::StateAttitude &attitude, const bool &updateFlag)
{
    currentAttitude = attitude;
    attitudeUpdated = updateFlag;
}

void Ardupilot_GeneralController::updateGlobalPositionTopic(const DataState::StateGlobalPosition &globalPosition, const bool &updateFlag)
{
    currentPosition = globalPosition;
    positionUpdated = updateFlag;
}

void Ardupilot_GeneralController::updateFlightMode(const DataARDUPILOT::VehicleFlightMode &flightMode, const bool &updateFlag)
{
    currentVehicleMode = flightMode;
    modeUpdated = updateFlag;
}

void Ardupilot_GeneralController::updatedHomePostion(const MissionItem::SpatialHome &homePosition, const bool &updateFlag)
{
     currentHome = homePosition;
}
*/

void Ardupilot_GeneralController::terminateObject()
{
    mToExit = true;
}
