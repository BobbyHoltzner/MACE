#include "vehicle_operating_attitude.h"

namespace DataVehicleArdupilot
{
    void VehicleOperatingAttitude::handleMSG(const mavlink_attitude_t &msg)
    {
        pitch = msg.pitch;
        roll = msg.roll;
        yaw = msg.yaw;

        pitchRate = msg.pitchspeed;
        rollRate = msg.rollspeed;
        yawRate = msg.yawspeed;
    }
}
