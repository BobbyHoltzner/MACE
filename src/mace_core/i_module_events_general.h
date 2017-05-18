#ifndef I_MODULE_EVENTS_GENERAL_H
#define I_MODULE_EVENTS_GENERAL_H

#include "data/system_description.h"
#include "data_generic_command_item/command_item_components.h"

namespace MaceCore
{

class IModuleEventsGeneral
{
public:

    virtual void Event_IssueCommandSystemArm(const void* sender, const CommandItem::ActionArm &command) = 0;

    virtual void Event_IssueCommandTakeoff(const void* sender, const CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> &command) = 0;

    virtual void Event_IssueCommandLand(const void* sender, const CommandItem::SpatialLand<DataState::StateGlobalPosition> &command) = 0;

    virtual void Event_IssueCommandRTL(const void* sender, const CommandItem::SpatialRTL &command) = 0;

    virtual void Event_ChangeSystemMode(const void* sender, const CommandItem::ActionChangeMode &command) = 0;

    virtual void Event_IssueGeneralCommand(const void* sender, const std::shared_ptr<CommandItem::AbstractCommandItem> &command) = 0;


    virtual void Event_GetMission(const void* sender, const Data::MissionKey &key) = 0;
    virtual void Event_GetOnboardMission(const void* sender, const int &systemID, const Data::MissionType &type) = 0;
    virtual void Event_GetCurrentMission(const void* sender, const int &systemID) = 0;

    virtual void RequestClearVehicleMission(const void* sender, const Data::SystemDescription &systemID) = 0;
    virtual void RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID) = 0;

    virtual void Event_GetHomePosition(const void* sender, const int &vehicleID) = 0;
    virtual void Event_SetHomePosition(const void* sender, const CommandItem::SpatialHome &vehicleHome) = 0;

    virtual void Event_SetGlobalOrigin(const void* sender, const CommandItem::SpatialHome &globalHome) = 0;
};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_GENERAL_H
