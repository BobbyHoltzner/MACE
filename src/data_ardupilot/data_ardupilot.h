#ifndef DATA_ARDUPILOT_H
#define DATA_ARDUPILOT_H

#include "data_ardupilot_global.h"

#include "mace_core/vehicle_object.h"
//#include "module_vehicle_MAVLINK/message_definition_mavlink.h"

class DATA_ARDUPILOTSHARED_EXPORT DataArdupilot : public VehicleObject
{

public:
    DataArdupilot(const int &vehicleID);
    virtual void handleMessage(VehicleMessage message) const;
};

#endif // DATA_ARDUPILOT_H
