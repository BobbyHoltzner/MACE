#ifndef DATA_GENERIC_ITEM_FLIGHTMODE_H
#define DATA_GENERIC_ITEM_FLIGHTMODE_H

#include <string>
#include "data/vehicle_types.h"
#include "data/autopilot_types.h"

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
        this->flightModeString = flightMode;
    }

    std::string getFlightModeString() const {
        return (flightModeString);
    }

    void setFlightMode(const int &flightMode) {
        this->flightModeInt = flightMode;
    }

    int getFlightModeInt() const {
        return (flightModeInt);
    }

    void setVehicleArmed(const bool armed){
        vehicleArmed = armed;
    }

    bool getVehicleArmed(){
        return(vehicleArmed);
    }

    void setAutopilotType(const Data::AutopilotTypes &type)
    {
        this->autopilotType = type;
    }

    Data::AutopilotTypes getAutopilotType() const
    {
        return(autopilotType);
    }

public:
    void operator = (const DataGenericItem_FlightMode &rhs)
    {
        this->vehicleType = rhs.vehicleType;
        this->autopilotType = rhs.autopilotType;
        this->flightModeString = rhs.flightModeString;
        this->flightModeInt = rhs.flightModeInt;
        this->vehicleArmed = rhs.vehicleArmed;
    }

    bool operator == (const DataGenericItem_FlightMode &rhs) {
        if(this->vehicleType != rhs.vehicleType){
            return false;
        }
        if(this->autopilotType != rhs.autopilotType){
            return false;
        }
        if(this->flightModeString != rhs.flightModeString) {
            return false;
        }
        if(this->flightModeInt != rhs.flightModeInt) {
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
    Data::AutopilotTypes autopilotType;
    std::string flightModeString;
    int flightModeInt;
    bool vehicleArmed;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_FLIGHTMODE_H
