#ifndef I_MODULE_EVENTS_GENERAL_H
#define I_MODULE_EVENTS_GENERAL_H

#include "data/system_description.h"
#include "data_generic_mission_item/mission_item_components.h"

namespace MaceCore
{

class IModuleEventsGeneral
{
public:
    virtual void RequestVehicleArm(const void* sender, const MissionItem::ActionArm &arm) = 0;
    virtual void RequestVehicleMode(const void* sender, const MissionItem::ActionChangeMode &changeMode) = 0;
    virtual void RequestVehicleTakeoff(const void* sender, const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff) = 0;

    //!
    //! \brief RequestSetVehicleMission method calls the appropriate handling operations to migrate the proposed
    //! mission list to the appropriate vehicle module for handling.
    //! \param sender
    //! \param missionList
    //!
    virtual void RequestSetVehicleMission(const void* sender, const MissionItem::MissionList &missionList) = 0;

    virtual void RequestVehicleMission(const void* sender, const Data::SystemDescription &systemID) = 0;
    virtual void RequestClearVehicleMission(const void* sender, const Data::SystemDescription &systemID) = 0;

    virtual void RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID) = 0;

    virtual void RequestVehicleHomePosition(const void* sender, const int &vehicleID) = 0;
    virtual void SetVehicleHomePosition(const void* sender, const MissionItem::SpatialHome &vehicleHome) = 0;

    virtual void UpdateGlobalOriginPosition(const void* sender, const MissionItem::SpatialHome &globalHome) = 0;
};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_GENERAL_H
