#ifndef ARDUPILOTSTATUS_H
#define ARDUPILOTSTATUS_H

#include <map>

#include "mavlink.h"

namespace Ardupilot{

class ArdupilotStatus
{
public:
    ArdupilotStatus();

    void setVehicleStatus(const int &status);

private:
    int statusCode;
    std::string statusDescription;
    std::string statusState;

    std::map<int, std::string> vehicleStatusDescrion = {{MAV_STATE_UNINIT,"Unitialized, booting up."},
                                               {MAV_STATE_BOOT,"Booting system, please wait."},
                                               {MAV_STATE_CALIBRATING,"Calibrating sensors, please wait."},
                                               {MAV_STATE_ACTIVE,"Active, normal operation."},
                                               {MAV_STATE_STANDBY,"Standby mode, ready for launch."},
                                               {MAV_STATE_CRITICAL,"FAILURE: Continuing operation."},
                                               {MAV_STATE_EMERGENCY,"EMERGENCY: Land Immediately!"}};

    std::map<int, std::string> vehicleStatusState = {{MAV_STATE_UNINIT,"UNINIT"},
                                               {MAV_STATE_BOOT,"BOOT"},
                                               {MAV_STATE_CALIBRATING,"CALIBRATING"},
                                               {MAV_STATE_ACTIVE,"ACTIVE"},
                                               {MAV_STATE_STANDBY,"STANDBY"},
                                               {MAV_STATE_CRITICAL,"CRITICAL"},
                                               {MAV_STATE_EMERGENCY,"EMERGENCY"}};
};
} //end of namespace Ardupilot

#endif // ARDUPILOTSTATUS_H
