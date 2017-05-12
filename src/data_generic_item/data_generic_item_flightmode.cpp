#include "data_generic_item_flightmode.h"

namespace DataGenericItem {

DataGenericItem_FlightMode::DataGenericItem_FlightMode():
    flightModeString("")
{

}

DataGenericItem_FlightMode::DataGenericItem_FlightMode(const mace_vehicle_mode_t &mode)
{
    this->flightModeString = std::string(mode.vehicle_mode);
}


DataGenericItem_FlightMode::DataGenericItem_FlightMode(const DataGenericItem_FlightMode &copyObj)
{
    this->flightModeString = copyObj.getFlightModeString();
}

} //end of namespace DataGenericItem
