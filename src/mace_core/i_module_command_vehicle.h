#ifndef I_MODULE_COMMAND_VEHICLE_H
#define I_MODULE_COMMAND_VEHICLE_H

#include "abstract_module_base_vehicle_listener.h"
#include "metadata_vehicle.h"

#include "i_module_topic_events.h"
#include "i_module_events_vehicle.h"

#include "data_generic_command_item/command_item_components.h"

namespace MaceCore
{

enum class VehicleCommands
{
    BASE_MODULE_VEHICLE_LISTENER_ENUMS,
    REQUEST_DUMMY_FUNCTION,
    UPDATE_MISSION_KEY,
    UPDATED_DYNAMIC_MISSION_QUEUE
};


class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandVehicle : public AbstractModule_VehicleListener<MetadataVehicle, IModuleEventsVehicle, VehicleCommands>
{
friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandVehicle():
        AbstractModule_VehicleListener()
    {
        //These are from MACE Core to modules
        this->template AddCommandLogic<int>(VehicleCommands::REQUEST_DUMMY_FUNCTION, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            RequestDummyFunction(vehicleID);
        });

        this->template AddCommandLogic<MissionItem::MissionKeyChange>(VehicleCommands::UPDATE_MISSION_KEY, [this](const MissionItem::MissionKeyChange &key, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UpdateMissionKey(key);
        });

        this->template AddCommandLogic<TargetItem::DynamicMissionQueue>(VehicleCommands::UPDATED_DYNAMIC_MISSION_QUEUE, [this](const TargetItem::DynamicMissionQueue &queue, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UpdateDynamicMissionQueue(queue);
        });


    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }


public:
    //!
    //! \brief RequestDummyFunction Test function
    //! \param vehicleID Vehicle ID
    //!
    virtual void RequestDummyFunction(const int &vehicleID) = 0;

    //!
    //! \brief UpdateMissionKey Update mission key command
    //! \param key Mission key to update
    //!
    virtual void UpdateMissionKey(const MissionItem::MissionKeyChange &key) = 0;

    //!
    //! \brief UpdateDynamicMissionQueue Update dynamic missio queue command
    //! \param queue Mission queue
    //!
    virtual void UpdateDynamicMissionQueue(const TargetItem::DynamicMissionQueue &queue) = 0;

};


} //END MaceCore Namespace

#endif // I_MODULE_COMMAND_VEHICLE_H
