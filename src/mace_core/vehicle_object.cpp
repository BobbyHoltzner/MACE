#include "vehicle_object.h"

VehicleObject::VehicleObject(const int &vehicleID)
{
    m_VehicleID = vehicleID;
}

VehicleObject::~VehicleObject()
{

}

int VehicleObject::getVehicleID() const
{
    return m_VehicleID;
}
