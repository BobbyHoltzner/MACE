#include "data_vehicle.h"


DataVehicle::DataVehicle(const int &vehicleID)
{
    m_VehicleID = vehicleID;
}

DataVehicle::~DataVehicle()
{

}

int DataVehicle::getVehicleID() const
{
    return m_VehicleID;
}


