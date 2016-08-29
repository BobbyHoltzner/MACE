#ifndef I_MODULE_COMMAND_PATH_PLANNING_H
#define I_MODULE_COMMAND_PATH_PLANNING_H

#include <string>
#include <map>

#include "abstract_module_base_vehicle_listener.h"
#include "metadata_path_planning.h"
#include "i_module_events_path_planning.h"



#include "metadata_vehicle.h"

namespace MaceCore
{


enum class PathPlanningCommands
{
    BASE_MODULE_VEHICLE_LISTENER_ENUMS,
    NEW_VEHICLE_TARGET,
    RECOMPUTE_PATHS
};

class IModuleCommandPathPlanning  : public AbstractModule_VehicleListener<MetadataPathPlanning, IModuleEventsPathPlanning, PathPlanningCommands>
{
public:

    static Classes moduleClass;

    IModuleCommandPathPlanning():
        AbstractModule_VehicleListener()
    {
        m_EventLooper.AddLambda<std::string>(PathPlanningCommands::NEW_VEHICLE_TARGET, [this](const std::string &vehicleID){
            NewVehicleTarget(vehicleID);
        });

        m_EventLooper.AddLambda(PathPlanningCommands::RECOMPUTE_PATHS, [this](){
            RecomputePaths();
        });
    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }


    //!
    //! \brief New targets have been assigned to the given vehicle
    //! \param vehicleID ID of vehicle
    //!
    virtual void NewVehicleTarget(const std::string &vehicleID) = 0;


    //!
    //! \brief For one reason or another a recomputation of all vehicles' paths is requested
    //!
    virtual void RecomputePaths() = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_PATH_PLANNING_H
