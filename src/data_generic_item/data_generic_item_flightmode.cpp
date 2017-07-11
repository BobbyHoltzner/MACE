#include "data_generic_item_flightmode.h"

namespace DataGenericItem {

DataGenericItem_FlightMode::DataGenericItem_FlightMode():
    flightModeString("")
{

}

DataGenericItem_FlightMode::DataGenericItem_FlightMode(const std::string &mode)
{
    this->flightModeString = mode;
}


DataGenericItem_FlightMode::DataGenericItem_FlightMode(const DataGenericItem_FlightMode &copyObj)
{
    this->flightModeString = copyObj.getFlightModeString();
}


mace_vehicle_mode_t DataGenericItem_FlightMode::getMACECommsObject() const
{
    mace_vehicle_mode_t rtnObj;

    strcpy(rtnObj.vehicle_mode,this->flightModeString.c_str());

    return rtnObj;
}

} //end of namespace DataGenericItem
