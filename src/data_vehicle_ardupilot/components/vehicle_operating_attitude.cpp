#include "vehicle_operating_attitude.h"

namespace DataVehicleArdupilot
{
    void VehicleOperatingAttitude::handleMSG(const mavlink_attitude_t &msg)
    {
        m_pitch = msg.pitch;
        m_roll = msg.roll;
        m_yaw = msg.yaw;

        m_pitchRate = msg.pitchspeed;
        m_rollRate = msg.rollspeed;
        m_yawRate = msg.yawspeed;
    }
}
