#ifndef DATA_ARDUPILOT_H
#define DATA_ARDUPILOT_H

#include "mavlink.h"
#include <string>
#include <Eigen/Dense>

#include "data_ardupilot_global.h"

#include "ardupilot_attitude.h"
#include "ardupilot_flightmode.h"
#include "ardupilot_position.h"
#include "ardupilot_status.h"

#include "data/global_position.h"
#include "data/local_position.h"

#include "mace_core/vehicle_object.h"
#include "module_vehicle_MAVLINK/generic_message_definition_mavlink.h"

namespace Ardupilot{

class DATA_ARDUPILOTSHARED_EXPORT DataArdupilot : public VehicleObject
{
public:
    DataArdupilot(const int &vehicleID, const int &vehicleProtocol, const int &vehicleType);

    DataArdupilot(DataArdupilot &copyObj);

    ~DataArdupilot();

    virtual void handleMessage(VehicleMessage msgIn);
    virtual void getVehicleMode(std::string &rtnString);
    virtual void getVehicleAttitude(Eigen::Vector3d &rtnString);
    virtual void getVehiclePosition(int &positionFix, int &numSats, Data::GlobalPosition &position);
    virtual void getVehicleFuel(Eigen::Vector2d &rtnVector);

//    virtual int getVehicleID() const;
//    virtual int getVehicleProtocol() const;
//    virtual int getVehicleType() const;

//public slots:
//    void newValue(double value);

private:
    ArdupilotFlightMode* m_FlightMode;
    ArdupilotAttitude* m_Attitude;
    ArdupilotStatus* m_Status;
    ArdupilotPosition* m_Position;

};
} //end of namespace ardupilot

#endif // DATA_ARDUPILOT_H
