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
    BASE_MODULE_LISTENER_ENUMS,
    NEWLY_UPDATED_OCCUPANCY_MAP,
    NEWLY_UPDATE_VEHICLE_BOUNDARIES
};

class MACE_CORESHARED_EXPORT IModuleCommandPathPlanning  : public AbstractModule_EventListeners<MetadataPathPlanning, IModuleEventsPathPlanning, PathPlanningCommands>
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandPathPlanning():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(PathPlanningCommands::NEWLY_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableVehicle(vehicleID);
        });

        AddCommandLogic<int>(PathPlanningCommands::NEWLY_UPDATED_OCCUPANCY_MAP, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UNUSED(vehicleID);
            NewlyUpdatedOccupancyMap();
        });

        AddCommandLogic<int>(PathPlanningCommands::NEWLY_UPDATED_GLOBAL_ORIGIN, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UNUSED(vehicleID);
            NewlyUpdatedGlobalOrigin();
        });

        AddCommandLogic<int>(PathPlanningCommands::NEWLY_UPDATE_VEHICLE_BOUNDARIES, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UNUSED(vehicleID);
            NewlyUpdateVehicleBoundaries();
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:
    //!
    //! \brief NewlyAvailableVehicle
    //! \param vehicleID
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;

    //!
    //! \brief NewlyUpdatedOccupancyMap
    //!
    virtual void NewlyUpdatedOccupancyMap() = 0;

    //!
    //! \brief NewlyUpdatedGlobalOrigin
    //!
    virtual void NewlyUpdatedGlobalOrigin() = 0;


    //!
    //! \brief NewlyUpdatedGlobalOrigin
    //!
    virtual void NewlyUpdateVehicleBoundaries() = 0;

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
