#ifndef I_GROUND_STATION_H
#define I_GROUND_STATION_H

#include "abstract_module_base_vehicle_listener.h"
#include "metadata_ground_station.h"

#include "i_module_topic_events.h"
#include "i_module_events_ground_station.h"

namespace MaceCore
{

enum class GroundStationCommands
{
    BASE_MODULE_VEHICLE_LISTENER_ENUMS,
    NEW_AVAILABLE_VEHICLE
};

class MACE_CORESHARED_EXPORT IModuleCommandGroundStation : public AbstractModule_EventListeners<Metadata_GroundStation, IModuleEventsGroundStation, GroundStationCommands>
{
    friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandGroundStation():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(GroundStationCommands::NEW_AVAILABLE_VEHICLE, [this](const int &vehicleID){
            NewlyAvailableVehicle(vehicleID);
        });
    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }

public:
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;

    virtual bool StartTCPServer() = 0;


};


} //End MaceCore Namespace

#endif // I_GROUND_STATION_H
