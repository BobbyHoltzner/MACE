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
    START_TCP_SERVER,
    NEW_VEHICLE_TARGET,
    NEW_AVAILABLE_VEHICLE
//    UPDATED_POSITION_DYNAMICS
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

    //!
    //! \brief Signal indicating the Occupancy Map has been updated
    //!
    //! The map data can be read from using MaceData object in getDataObject()
    //!
    virtual bool StartTCPServer() = 0;


//    //!
//    //! \brief New targets have been assigned to the given vehicle
//    //! \param vehicleID ID of vehicle
//    //!
//    virtual void NewVehicleTarget(const std::string &vehicleID) = 0;

};


} //End MaceCore Namespace

#endif // I_GROUND_STATION_H
