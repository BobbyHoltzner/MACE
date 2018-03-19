#ifndef I_MODULE_COMMAND_PATH_PLANNING_H
#define I_MODULE_COMMAND_PATH_PLANNING_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_path_planning.h"

#include "i_module_topic_events.h"
#include "i_module_events_path_planning.h"

namespace MaceCore
{

enum class PathPlanningCommands
{
    NEW_AVAILABLE_VEHICLE
};

class MACE_CORESHARED_EXPORT IModuleCommandPathPlanning  : public AbstractModule_EventListeners<MetadataPathPlanning, IModuleEventsPathPlanning, PathPlanningCommands>
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandPathPlanning():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(PathPlanningCommands::NEW_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableVehicle(vehicleID);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;

//    //!
//    //! \brief New targets have been assigned to the given vehicle
//    //! \param vehicleID ID of vehicle
//    //!
//    virtual void NewVehicleTarget(const std::string &vehicleID) = 0;


//    //!
//    //! \brief For one reason or another a recomputation of all vehicles' paths is requested
//    //!
//    virtual void RecomputePaths() = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_PATH_PLANNING_H
