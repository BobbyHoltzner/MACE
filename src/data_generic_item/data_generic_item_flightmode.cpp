#include "data_generic_item_flightmode.h"

namespace DataGenericItem {

DataGenericItem_FlightMode::DataGenericItem_FlightMode():
    vehicleType(Data::VehicleTypes::UNKNOWN), autopilotType(Data::AutopilotTypes::UNKNOWN), flightModeString(""), flightModeInt(0), vehicleArmed(false)
{

}

DataGenericItem_FlightMode::DataGenericItem_FlightMode(const DataGenericItem_FlightMode &copyObj)
{
    this->vehicleType = copyObj.getVehicleType();
    this->autopilotType = copyObj.getAutopilotType();
    this->flightModeString = copyObj.getFlightModeString();
    this->flightModeInt = copyObj.getFlightModeInt();
    this->vehicleArmed = copyObj.getVehicleArmed();
}

} //end of namespace DataGenericItem
