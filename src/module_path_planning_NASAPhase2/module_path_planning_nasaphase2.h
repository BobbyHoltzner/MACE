#ifndef MODULE_PATH_PLANNING_NASAPHASE2_H
#define MODULE_PATH_PLANNING_NASAPHASE2_H

#include "module_path_planning_nasaphase2_global.h"

#include "mace_core/i_module_command_path_planning.h"

class ModulePathPlanningNASAPhase2 : public MaceCore::IModuleCommandPathPlanning
{

public:
    ModulePathPlanningNASAPhase2(const MaceCore::MetadataPathPlanning metaData);

public:


    virtual void NewVehicle(const std::string &ID, const MaceCore::MetadataVehicle &vehicle);

    virtual void RemoveVehicle(const std::string &ID);

    virtual void UpdatedPosition(const std::string &vehicleID);

    virtual void UpdateDynamicsState(const std::string &vehicleID);

    virtual void UpdatedVehicleLife(const std::string &vehicleID);
};

#endif // MODULE_PATH_PLANNING_NASAPHASE2_H
