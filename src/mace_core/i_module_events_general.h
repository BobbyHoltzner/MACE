#ifndef I_MODULE_EVENTS_GENERAL_H
#define I_MODULE_EVENTS_GENERAL_H

#include "data/system_description.h"
#include "data_generic_mission_item/mission_item_components.h"

namespace MaceCore
{

class IModuleEventsGeneral
{
public:
    //!
    //! \brief Event_ArmVehicle
    //! \param sender
    //! \param arm
    //!
    virtual void Event_ArmVehicle(const void* sender, const MissionItem::ActionArm &arm) = 0;

    //!
    //! \brief Event_ChangeVehicleMode
    //! \param sender
    //! \param changeMode
    //!
    virtual void Event_ChangeVehicleMode(const void* sender, const MissionItem::ActionChangeMode &changeMode) = 0;

    //!
    //! \brief Command_RequestVehicleTakeoff
    //! \param sender
    //! \param vehicleTakeoff
    //!
    virtual void Event_RequestVehicleTakeoff(const void* sender, const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff) = 0;

    //virtual void Event_GetMission(const void* sender, const int &systemID) = 0;

    virtual void Event_GetMission(const void* sender, const Data::MissionKey &key) = 0;
    virtual void Event_GetOnboardMission(const void* sender, const int &systemID, const Data::MissionType &type) = 0;
    virtual void Event_GetCurrentMission(const void* sender, const int &systemID) = 0;

    virtual void RequestClearVehicleMission(const void* sender, const Data::SystemDescription &systemID) = 0;
    virtual void RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID) = 0;

    virtual void Event_GetHomePosition(const void* sender, const int &vehicleID) = 0;
    virtual void Event_SetHomePosition(const void* sender, const MissionItem::SpatialHome &vehicleHome) = 0;

    virtual void Event_SetGlobalOrigin(const void* sender, const MissionItem::SpatialHome &globalHome) = 0;
};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_GENERAL_H
