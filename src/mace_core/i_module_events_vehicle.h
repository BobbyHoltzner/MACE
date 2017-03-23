#ifndef I_VEHICLE_EVENTS_H
#define I_VEHICLE_EVENTS_H
#include "data/mission_map.h"
#include "data_generic_state_item/state_item_components.h"
#include "data_generic_mission_item/mission_item_components.h"

namespace MaceCore
{

class IModuleEventsVehicle
{
public:
    //A vehicle module can indicate something has happened
    virtual void NewConstructedVehicle(const void* sender, const int &newVehicleObserved) = 0;
    virtual void NewVehicleHomePosition(const void* sender, const MissionItem::SpatialHome &vehicleHome) = 0;
    virtual void UpdateVehicleMission(const void* sender, const MissionItem::MissionList::MissionListState status, const MissionItem::MissionList &missionList, const Data::MissionMap &relevantMissionProfile) = 0;
};

} //End MaceCore Namespace

#endif // I_VEHICLE_EVENTS_H
