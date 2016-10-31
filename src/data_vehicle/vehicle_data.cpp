#include "vehicle_data.h"

using namespace Data;

VehicleData::VehicleData(){

}

VehicleData::~VehicleData(){

}

int VehicleData::getSendingID()
{
    return m_VehicleID;
}

void VehicleData::setSendingID(const int &id)
{
    m_VehicleID = id;
}
