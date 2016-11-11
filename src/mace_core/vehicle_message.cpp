#include "vehicle_message.h"

//namespace MaceCore {
//AbstractVehicleMessage::AbstractVehicleMessage(const int &vehicleID)
//{
//    m_VehicleID = vehicleID;
//}
AbstractVehicleMessage::~AbstractVehicleMessage()
{

}

int AbstractVehicleMessage::getVehicleID() const
{
    return m_VehicleID;
}

VehicleMessage::VehicleMessage()
{

}

void VehicleMessage::setDataObject(const std::shared_ptr<AbstractVehicleMessage> &data)
{
    this->messageData = data;
}

std::shared_ptr<AbstractVehicleMessage> VehicleMessage::getDataObject() const
{
    return this->messageData;
}

//} //end of namespace MaceCore
