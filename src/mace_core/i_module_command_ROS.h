#ifndef I_MODULE_COMMAND_ROS_H
#define I_MODULE_COMMAND_ROS_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_ROS.h"

#include "i_module_topic_events.h"
#include "i_module_events_ROS.h"

namespace MaceCore
{

enum class ROSCommands
{
    NEW_AVAILABLE_VEHICLE
};

class MACE_CORESHARED_EXPORT IModuleCommandROS  : public AbstractModule_EventListeners<MetadataROS, IModuleEventsROS, ROSCommands>
{
    friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandROS():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(ROSCommands::NEW_AVAILABLE_VEHICLE, [this](const int &vehicleID){
            NewlyAvailableVehicle(vehicleID);
        });
    }

    virtual Classes ModuleClass() const
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


};

} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_ROS_H
