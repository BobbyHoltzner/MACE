#include "ardupilot_status.h"
namespace Ardupilot{
ArdupilotStatus::ArdupilotStatus()
{
    statusState = "UNKNOWN";
    statusDescription = "Unknown system state.";
}

void ArdupilotStatus::setVehicleStatus(const int &status)
{
    this->statusCode = status;
}

void ArdupilotStatus::updatedVehicleHeartbeat(const mavlink_heartbeat_t msg)
{

}

} //end of namespace Ardupilot
