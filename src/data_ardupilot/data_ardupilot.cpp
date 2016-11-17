#include "data_ardupilot.h"

namespace Ardupilot {

DataArdupilot::DataArdupilot(const int &vehicleID, const int &vehicleProtocol, const int &vehicleType)
    :VehicleObject(vehicleID,vehicleProtocol,vehicleType)
{
    m_FlightMode = new ArdupilotFlightMode();
    m_FlightMode->setVehicleType(vehicleType);

}

void DataArdupilot::handleMessage(VehicleMessage message) const
{
    std::shared_ptr<AbstractVehicleMessage> tmpAbstractMessage = message.getDataObject();
    std::string MsgType = tmpAbstractMessage->getMessageType();
}

} //end of namespace Ardupilot
