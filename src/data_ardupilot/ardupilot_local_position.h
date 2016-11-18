#ifndef ARDUPILOTLOCALPOSITIONING_H
#define ARDUPILOTLOCALPOSITIONING_H

#include "mavlink.h"

namespace Ardupilot{

class ArdupilotLocalPosition
{
public:
    ArdupilotLocalPosition();

    //ArdupilotLocalPosition(const ArdupilotLocalPosition &copyObj);

    void updateFromMavlink(const mavlink_local_position_ned_t &localPositionMSG);

    bool operator ==(const ArdupilotLocalPosition &rhs) const;

    bool operator !=(const ArdupilotLocalPosition &rhs) const;

public:
    double xPosition;
    double yPosition;
    double zPosition;
    double xVelocity;
    double yVelocity;
    double zVelocity;
};

} //end of namespace Ardupilot
#endif // ARDUPILOTLOCALPOSITIONING_H
