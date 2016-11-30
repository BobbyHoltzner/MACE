#ifndef ARDUPILOTGLOBALPOSITION_H
#define ARDUPILOTGLOBALPOSITION_H

#include <Eigen/Dense>
#include "mavlink.h"

namespace Ardupilot{

struct GlobalPositionStruct{
    double latitude;
    double longitude;
    double zPosition;
    double xVelocity;
    double yVelocity;
    double zVelocity;
    double xAcceleration;
    double yAcceleration;
    double zAcceleration;
    double yaw;
    double yawRate;

    GlobalPositionStruct();

    bool operator ==(const GlobalPositionStruct &rhs) const;

    bool operator !=(const GlobalPositionStruct &rhs) const;
};

class ArdupilotGlobalPosition
{


public:
    ArdupilotGlobalPosition();

    //ArdupilotGlobalPosition(const ArdupilotGlobalPosition &copyObj);

    void updateFromMavlink(const mavlink_global_position_int_t &globalPositionMSG);

    void getGlobalPosition(Eigen::Vector3d &positionVector);

    bool operator ==(const ArdupilotGlobalPosition &rhs) const;

    bool operator !=(const ArdupilotGlobalPosition &rhs) const;

public:
    double latitude;
    double longitude;
    double altitude;
    double altitude_relative;
    double groundSpeed_X;
    double groundSpeed_Y;
    double groundSpeed_Z;
    double heading;

};

} //end of namespace Ardupilot
#endif // ARDUPILOTGLOBALPOSITION_H
