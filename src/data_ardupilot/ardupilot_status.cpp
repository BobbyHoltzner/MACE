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
} //end of namespace Ardupilot
