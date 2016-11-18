#ifndef ARDUPILOTGLOBALPOSITION_H
#define ARDUPILOTGLOBALPOSITION_H

#include "mavlink.h"

namespace Ardupilot{

class ArdupilotGlobalPosition
{
public:
    ArdupilotGlobalPosition();

    //ArdupilotGlobalPosition(const ArdupilotGlobalPosition &copyObj);

    void updateFromMavlink(const mavlink_global_position_int_t &globalPositionMSG);

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
