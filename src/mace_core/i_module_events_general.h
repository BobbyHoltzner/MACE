#ifndef I_MODULE_EVENTS_GENERAL_H
#define I_MODULE_EVENTS_GENERAL_H

#include "base/pose/geodetic_position_3D.h"

#include "data/system_description.h"
#include "data_generic_command_item/command_item_components.h"
#include "data_generic_state_item/state_global_position.h"

#include "abstract_module_base.h"

namespace MaceCore
{

class IModuleEventsGeneral
{
public:

    virtual void Event_ForceVehicleDataSync(const ModuleBase* sender, const int &targetSystemID) = 0;

    virtual void Event_IssueCommandSystemArm(const ModuleBase* sender, const CommandItem::ActionArm &command) = 0;

    virtual void Event_IssueCommandTakeoff(const ModuleBase* sender, const CommandItem::SpatialTakeoff &command) = 0;

    virtual void Event_IssueCommandLand(const ModuleBase* sender, const CommandItem::SpatialLand &command) = 0;

    virtual void Event_IssueCommandRTL(const ModuleBase* sender, const CommandItem::SpatialRTL &command) = 0;

    virtual void Event_IssueMissionCommand(const ModuleBase* sender, const CommandItem::ActionMissionCommand &command) = 0;

    virtual void Event_ChangeSystemMode(const ModuleBase *sender, const CommandItem::ActionChangeMode &command) = 0;

    virtual void Event_IssueGeneralCommand(const ModuleBase* sender, const std::shared_ptr<CommandItem::AbstractCommandItem> &command) = 0;

    virtual void Event_GetMission(const void* sender, const MissionItem::MissionKey &key) = 0;
    virtual void Event_GetOnboardMission(const void* sender, const int &systemID, const MissionItem::MISSIONTYPE &type) = 0;
    virtual void Event_GetCurrentMission(const void* sender, const int &systemID) = 0;

    virtual void RequestClearVehicleMission(const void* sender, const Data::SystemDescription &systemID) = 0;
    virtual void RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID) = 0;

    virtual void Event_GetHomePosition(const void* sender, const int &vehicleID) = 0;
    virtual void Event_SetHomePosition(const ModuleBase *sender, const CommandItem::SpatialHome &vehicleHome) = 0;

    virtual void Event_SetGlobalOrigin(const void* sender, const mace::pose::CartesianPosition_3D &globalHome) = 0;
    virtual void Event_SetGridSpacing(const void* sender, const double &gridSpacing) = 0;

    virtual void Event_SetOperationalBoundary(const ModuleBase *sender, const BoundaryItem::BoundaryList &boundary) = 0;

    virtual void Event_SetEnvironmentVertices(const ModuleBase *sender, const std::vector<DataState::StateGlobalPosition> &boundaryVerts) = 0;
};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_GENERAL_H
