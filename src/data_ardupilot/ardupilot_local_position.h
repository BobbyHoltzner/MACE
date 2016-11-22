#ifndef ARDUPILOTLOCALPOSITIONING_H
#define ARDUPILOTLOCALPOSITIONING_H

#include "mavlink.h"

namespace Ardupilot{

struct LocalPositionStruct{
    double xPosition;
    double yPosition;
    double zPosition;
    double xVelocity;
    double yVelocity;
    double zVelocity;
    double xAcceleration;
    double yAcceleration;
    double zAcceleration;
    double yaw;
    double yawRate;

    LocalPositionStruct();

    bool operator ==(const LocalPositionStruct &rhs) const;

    bool operator !=(const LocalPositionStruct &rhs) const;
};

class ArdupilotLocalPosition
{
public:
    ArdupilotLocalPosition();

    //ArdupilotLocalPosition(const ArdupilotLocalPosition &copyObj);

    void updateFromMavlink(const mavlink_local_position_ned_t &localPositionMSG);

    bool operator ==(const ArdupilotLocalPosition &rhs) const;

    bool operator !=(const ArdupilotLocalPosition &rhs) const;

public:

    //should house the current commanded local position target and the current target local
};

} //end of namespace Ardupilot
#endif // ARDUPILOTLOCALPOSITIONING_H
