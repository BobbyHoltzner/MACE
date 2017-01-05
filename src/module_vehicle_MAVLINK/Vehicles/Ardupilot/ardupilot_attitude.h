#ifndef ARDUPILOTATTITUDE_H
#define ARDUPILOTATTITUDE_H

#include <Eigen/Dense>
#include <mavlink.h>

namespace Ardupilot{

class ArdupilotAttitude
{
public:
    ArdupilotAttitude();
    ArdupilotAttitude(const ArdupilotAttitude &copyObject);

    void updateAttitudeMavlink(const mavlink_attitude_t &msgData);

    void getAttitudeComplete(Eigen::Vector3d &attitudeVector, Eigen::Vector3d &attitudeRateVector);
    void getAttitude(Eigen::Vector3d &attitudeVector);
    void getAttitudeRates(Eigen::Vector3d &attitudeRateVector);


private:
    double roll;
    double roll_rate;
    double pitch;
    double pitch_rate;
    double yaw;
    double yaw_rate;

};
} //end of namespace Ardupilot

#endif // ARDUPILOTATTITUDE_H
