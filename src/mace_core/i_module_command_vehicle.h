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
    UPDATE_MISSION_KEY
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

    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }


public:
    virtual void RequestDummyFunction(const int &vehicleID) = 0;
    virtual void UpdateMissionKey(const MissionItem::MissionKeyChange &key) = 0;
};


} //END MaceCore Namespace

#endif // I_MODULE_COMMAND_VEHICLE_H
