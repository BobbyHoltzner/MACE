#include "ardupilot_attitude.h"
namespace Ardupilot{

ArdupilotAttitude::ArdupilotAttitude()
{
    roll = 0.0;
    roll_rate = 0.;

    pitch = 0.0;
    pitch_rate = 0.0;

    yaw = 0.0;
    yaw_rate = 0.0;
}
void ArdupilotAttitude::updateAttitudeMavlink(const mavlink_attitude_t &msgData)
{
    roll = msgData.roll;
    roll_rate = msgData.rollspeed;

    pitch = msgData.pitch;
    pitch_rate = msgData.pitchspeed;

    yaw = msgData.yaw;
    yaw_rate = msgData.yawspeed;

    //emit valueChanged(roll);

}
} //end of namespace Ardupilot
