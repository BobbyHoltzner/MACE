#ifndef I_GROUND_STATION_H
#define I_GROUND_STATION_H

#include "abstract_module_event_listeners.h"
#include "metadata_ground_station.h"

#include "i_module_topic_events.h"
#include "i_module_events_ground_station.h"

#include "data_generic_command_item/command_item_components.h"

#include "base/pose/cartesian_position_3D.h"


#include "i_module_command_generic_boundaries.h"

namespace MaceCore
{

enum class GroundStationCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_BOUNDARY_LISTENER_ENUMS,
    NEWLY_AVAILABLE_CURRENT_MISSION,
    NEW_MISSION_EXE_STATE,
    NEWLY_AVAILABLE_HOME_POSITION
};

class MACE_CORESHARED_EXPORT IModuleCommandGroundStation :
        public AbstractModule_EventListeners<Metadata_GroundStation, IModuleEventsGroundStation, GroundStationCommands>,
        public IModuleGenericBoundaries
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandGroundStation():
        AbstractModule_EventListeners()
    {
        IModuleGenericBoundaries::SetUp<Metadata_GroundStation, IModuleEventsGroundStation, GroundStationCommands>(this);

        AddCommandLogic<int>(GroundStationCommands::NEWLY_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableVehicle(vehicleID);
        });

        AddCommandLogic<MissionItem::MissionKey>(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION, [this](const MissionItem::MissionKey &missionKey, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableCurrentMission(missionKey);
        });

        AddCommandLogic<MissionItem::MissionKey>(GroundStationCommands::NEW_MISSION_EXE_STATE, [this](const MissionItem::MissionKey &missionKey, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableMissionExeState(missionKey);
        });

        AddCommandLogic<CommandItem::SpatialHome>(GroundStationCommands::NEWLY_AVAILABLE_HOME_POSITION, [this](const CommandItem::SpatialHome &home, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableHomePosition(home, sender);
        });

        AddCommandLogic<mace::pose::GeodeticPosition_3D>(GroundStationCommands::NEWLY_UPDATED_GLOBAL_ORIGIN, [this](const mace::pose::GeodeticPosition_3D &position, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyUpdatedGlobalOrigin(position);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:
    //!
    //! \brief NewlyAvailableVehicle New available vehicle subscriber
    //! \param vehicleID New vehicle ID
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;

    //!
    //! \brief NewlyAvailableBoundary New boundary available subscriber
    //! \param key
    //!
    virtual void NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

    //!
    //! \brief NewlyAvailableCurrentMission New current mission available
    //! \param missionKey Mission key of new mission
    //!
    virtual void NewlyAvailableCurrentMission(const MissionItem::MissionKey &missionKey) = 0;

    //!
    //! \brief NewlyAvailableMissionExeState New mission EXE state available subscriber
    //! \param missionKey Mission key for new exe state
    //!
    virtual void NewlyAvailableMissionExeState(const MissionItem::MissionKey &missionKey) = 0;

    //!
    //! \brief NewlyAvailableHomePosition New home position available subscriber
    //! \param home Home position
    //! \param sender Sender module
    //!
    virtual void NewlyAvailableHomePosition(const CommandItem::SpatialHome &home, const OptionalParameter<ModuleCharacteristic> &sender) = 0;

    //!
    //! \brief NewlyUpdatedGlobalOrigin New global origin subscriber
    //! \param position New global origin position
    //!
    virtual void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) = 0;

    //!
    //! \brief StartTCPServer Start module TCP server for GUI communications
    //! \return
    //!
    virtual bool StartTCPServer() = 0;


};


} //End MaceCore Namespace

#endif // I_GROUND_STATION_H
