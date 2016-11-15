#ifndef DATA_ARDUPILOT_H
#define DATA_ARDUPILOT_H

#include "data_ardupilot_global.h"
#include "ardupilot_flightmode.h"

#include "mace_core/vehicle_object.h"

//#include "module_vehicle_MAVLINK/message_definition_mavlink.h"
namespace Ardupilot{

class DATA_ARDUPILOTSHARED_EXPORT DataArdupilot : public VehicleObject
{

public:
    DataArdupilot(const int &vehicleID, const int &vehicleProtocol, const int &vehicleType);

    virtual void handleMessage(VehicleMessage message) const;

private:
    ArdupilotFlightMode* m_FlightMode;

};
} //end of namespace ardupilot

#endif // DATA_ARDUPILOT_H
