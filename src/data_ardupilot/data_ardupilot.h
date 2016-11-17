#ifndef DATA_ARDUPILOT_H
#define DATA_ARDUPILOT_H

#include "mavlink.h"

#include "data_ardupilot_global.h"

#include "ardupilot_attitude.h"
#include "ardupilot_flightmode.h"
#include "ardupilot_status.h"

#include "mace_core/vehicle_object.h"
#include "module_vehicle_MAVLINK/generic_message_definition_mavlink.h"

namespace Ardupilot{

class DATA_ARDUPILOTSHARED_EXPORT DataArdupilot : public VehicleObject
{
public:
    DataArdupilot(const int &vehicleID, const int &vehicleProtocol, const int &vehicleType);

    DataArdupilot(DataArdupilot &copyObj);

    ~DataArdupilot();

    virtual void handleMessage(VehicleMessage msgIn) const;
//    virtual int getVehicleID() const;
//    virtual int getVehicleProtocol() const;
//    virtual int getVehicleType() const;

public slots:
    void newValue(double value);

private:
    ArdupilotFlightMode* m_FlightMode;
    ArdupilotAttitude* m_Attitude;
    ArdupilotStatus* m_Status;


};
} //end of namespace ardupilot

#endif // DATA_ARDUPILOT_H
