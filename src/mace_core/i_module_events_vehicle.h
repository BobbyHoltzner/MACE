#ifndef I_MODULE_VEHICLE_EVENTS_H
#define I_MODULE_VEHICLE_EVENTS_H

#include "i_module_events_general_vehicle.h"

namespace MaceCore
{

class IModuleEventsVehicle : public IModuleEventsGeneralVehicle
{
public:
    //A vehicle module can indicate something has happened

    //!
    //! \brief EventVehcile_NewOnboardVehicleMission This virtual function event is used to indicate that a vehicle has a new
    //! onboard available mission. The type will be described in the key of the MissionItem
    //! \param sender
    //! \param missionList
    //!
    virtual void EventVehicle_NewOnboardVehicleMission(const void *sender, const MissionItem::MissionList &missionList) = 0;
};

} //End MaceCore Namespace

#endif // I_MODULE_VEHICLE_EVENTS_H
