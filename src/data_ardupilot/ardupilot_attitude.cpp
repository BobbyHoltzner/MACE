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
void ArdupilotAttitude::updateAttitudeMavlink(mavlink_attitude_t msgAttitude)
{
    roll = msgAttitude.roll;
    roll_rate = msgAttitude.rollspeed;

    pitch = msgAttitude.pitch;
    pitch_rate = msgAttitude.pitchspeed;

    yaw = msgAttitude.yaw;
    yaw_rate = msgAttitude.yawspeed;

    emit valueChanged(roll);

}
} //end of namespace Ardupilot
