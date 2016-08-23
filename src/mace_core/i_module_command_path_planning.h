#ifndef I_MODULE_COMMAND_PATH_PLANNING_H
#define I_MODULE_COMMAND_PATH_PLANNING_H

#include <string>
#include <map>

#include "module_base.h"
#include "metadata_path_planning.h"
#include "i_module_events_path_planning.h"

#include "metadata_vehicle.h"

class IModuleCommandPathPlanning  : public ModuleBase<MetadataPathPlanning, IModuleEventsPathPlanning>
{
public:

    IModuleCommandPathPlanning(MetadataPathPlanning metadata):
        ModuleBase(metadata)
    {

    }

    virtual std::string ModuleName() const
    {
        return "PathPlanning";
    }

public:

    virtual void NewVehicle(const std::string &ID, const MetadataVehicle &vehicle) = 0;

    virtual void RemoveVehicle(const std::string &ID) = 0;

    virtual void UpdatedPosition(const std::string &vehicleID) = 0;

    virtual void UpdateDynamicsState(const std::string &vehicleID) = 0;

    virtual void UpdatedVehicleLife(const std::string &vehicleID) = 0;
};

#endif // I_MODULE_COMMAND_PATH_PLANNING_H