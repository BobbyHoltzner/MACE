#ifndef I_MODULE_COMMAND_VEHICLE_H
#define I_MODULE_COMMAND_VEHICLE_H

#include "abstract_module_base_vehicle_listener.h"
#include "metadata_vehicle.h"

#include "i_module_topic_events.h"
#include "i_module_events_vehicle.h"
#include "data/mission_key_change.h"
#include "data/mission_ack.h"

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

    static Classes moduleClass;

    IModuleCommandVehicle():
        AbstractModule_VehicleListener()
    {
        //These are from MACE Core to modules
        this->template AddCommandLogic<int>(VehicleCommands::REQUEST_DUMMY_FUNCTION, [this](const int &vehicleID){
            RequestDummyFunction(vehicleID);
        });

        this->template AddCommandLogic<Data::MissionKeyChange>(VehicleCommands::UPDATE_MISSION_KEY, [this](const Data::MissionKeyChange &key){
            UpdateMissionKey(key);
        });

    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }


public:
    virtual void RequestDummyFunction(const int &vehicleID) = 0;
    virtual void UpdateMissionKey(const Data::MissionKeyChange &key) = 0;
};


} //END MaceCore Namespace

#endif // I_MODULE_COMMAND_VEHICLE_H
