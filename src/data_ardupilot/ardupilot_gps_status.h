#ifndef ARDUPILOTGPSSTATUS_H
#define ARDUPILOTGPSSTATUS_H

#include "mavlink.h"

namespace Ardupilot{

class ArdupilotGPSStatus
{
public:
    ArdupilotGPSStatus();

    void updateFromMavlink(const mavlink_gps_status_t &localPositionMSG);

private:
    int fixStatus;
    int numberOfSats;
    int horizontalDOP;
    int verticalDOP;


};

} //end of namespace Ardupilot

#endif // ARDUPILOTGPSSTATUS_H
