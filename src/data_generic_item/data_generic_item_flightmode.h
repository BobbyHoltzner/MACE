#ifndef DATA_GENERIC_ITEM_FLIGHTMODE_H
#define DATA_GENERIC_ITEM_FLIGHTMODE_H

#include <string>
#include "data/vehicle_types.h"

namespace DataGenericItem {

class DataGenericItem_FlightMode
{

public:
    DataGenericItem_FlightMode();

public:

    void setVehicleType(const Data::VehicleTypes &vehicleType){
        this->vehicleType = vehicleType;
    }

    Data::VehicleTypes getVehicleType() const
    {
        return (vehicleType);
    }

    void setFlightMode(const std::string &flightMode) {
        this->flightMode = flightMode;
    }

    std::string getFlightMode() const {
        return (flightMode);
    }

    void setVehicleArmed(const bool armed){
        vehicleArmed = armed;
    }

    bool getVehicleArmed(){
        return(vehicleArmed);
    }

public:
    void operator = (const DataGenericItem_FlightMode &rhs)
    {
        this->vehicleType = rhs.vehicleType;
        this->flightMode = rhs.flightMode;
        this->vehicleArmed = rhs.vehicleArmed;
    }

    bool operator == (const DataGenericItem_FlightMode &rhs) {
        if(this->vehicleType != rhs.vehicleType){
            return false;
        }
        if(this->flightMode != rhs.flightMode) {
            return false;
        }
        if(this->vehicleArmed != rhs.vehicleArmed) {
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_FlightMode &rhs) {
        return !(*this == rhs);
    }

protected:
    Data::VehicleTypes vehicleType;
    std::string flightMode;
    bool vehicleArmed;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_FLIGHTMODE_H
