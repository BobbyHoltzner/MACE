#ifndef I_MODULE_COMMAND_ROS_H
#define I_MODULE_COMMAND_ROS_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_ROS.h"

#include "i_module_topic_events.h"
#include "i_module_events_ROS.h"

#include "maps/data_2d_grid.h"
#include "maps/octomap_wrapper.h"

namespace MaceCore
{

enum class ROSCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    NEWLY_COMPRESSED_OCCUPANCY_MAP,
    NEWLY_FOUND_PATH,
    TEST_FIRE
};

class MACE_CORESHARED_EXPORT IModuleCommandROS  : public AbstractModule_EventListeners<MetadataROS, IModuleEventsROS, ROSCommands>
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandROS():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(ROSCommands::NEWLY_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableVehicle(vehicleID);
        });
        AddCommandLogic<mace::maps::Data2DGrid<mace::maps::OctomapWrapper::OccupiedResult>>(ROSCommands::NEWLY_COMPRESSED_OCCUPANCY_MAP, [this](const mace::maps::Data2DGrid<mace::maps::OctomapWrapper::OccupiedResult> &map, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyCompressedOccupancyMap(map);
        });
        AddCommandLogic<std::vector<mace::state_space::StatePtr>>(ROSCommands::NEWLY_FOUND_PATH, [this](const std::vector<mace::state_space::StatePtr> &path, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyFoundPath(path);
        });
        AddCommandLogic<int>(ROSCommands::TEST_FIRE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UNUSED(vehicleID);
            TestFiring();
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:
    virtual void TestFiring() = 0;

    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;

    virtual void NewlyCompressedOccupancyMap(const mace::maps::Data2DGrid<mace::maps::OctomapWrapper::OccupiedResult> &map) = 0;

    virtual void NewlyFoundPath(const std::vector<mace::state_space::StatePtr> &path) = 0;

    //    //!
//    //! \brief New targets have been assigned to the given vehicle
//    //! \param vehicleID ID of vehicle
//    //!
//    virtual void NewVehicleTarget(const std::string &vehicleID) = 0;


};

} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_ROS_H
