#ifndef I_GROUND_STATION_H
#define I_GROUND_STATION_H

#include <string>
#include <map>

#include "abstract_module_base_vehicle_listener.h"
#include "metadata_ground_station.h"

#include "i_module_events_ground_station.h"

#include "metadata_ground_station.h"

#include "vehicle_data.h"

namespace MaceCore
{

enum class GroundStationCommands
{
    BASE_MODULE_VEHICLE_LISTENER_ENUMS,
    START_TCP_SERVER,
    NEW_VEHICLE_TARGET,
//    UPDATED_POSITION_DYNAMICS
};

class MACE_CORESHARED_EXPORT IModuleCommandGroundStation : public AbstractModule_VehicleListener<Metadata_GroundStation, IModuleEventsGroundStation, GroundStationCommands>
{
public:

    static Classes moduleClass;

    IModuleCommandGroundStation():
        AbstractModule_VehicleListener()
    {
//        AddCommandLogic(GroundStationCommands::START_TCP_SERVER, [this](){
//            StartTCPServer();
//        });

//        AddCommandLogic<std::string>(GroundStationCommands::NEW_VEHICLE_TARGET, [this](const std::string &vehicleID){
//            NewVehicleTarget(vehicleID);
//        });
    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }

public:




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
