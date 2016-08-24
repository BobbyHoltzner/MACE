#ifndef I_MODULE_EVENTS_PATH_PLANNING_H
#define I_MODULE_EVENTS_PATH_PLANNING_H

#include <string>
#include <vector>

#include "vehicle_data.h"

namespace MaceCore
{

class IModuleEventsPathPlanning
{
public:



    virtual void ReplaceVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands) = 0;

    virtual void ReplaceAfterCurrentVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands) = 0;

    virtual void AppendVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_PATH_PLANNING_H
