#include "vehicle_state_data.h"
using namespace Data;

VehicleStateData::VehicleStateData()
{

}

VehicleStateData::~VehicleStateData()
{

}

std::shared_ptr<VehicleData> VehicleStateData::getVehicleData() const
{
    return this->m_VehicleData;
}

void VehicleStateData::setVehicleData(const std::shared_ptr<VehicleData> &vehicleData)
{
    this->m_VehicleData = vehicleData;
}
