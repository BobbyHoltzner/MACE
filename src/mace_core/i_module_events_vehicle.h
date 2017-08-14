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
    //! \brief ExternalEvent_NewConstructedVehicle
    //! \param sender
    //! \param newVehicleObserved
    //!
    virtual void EventVehicle_NewConstructedVehicle(const void *sender, const int &newVehicleObserved) = 0;

    //!
    //! \brief EventVehcile_NewOnboardVehicleMission This virtual function event is used to indicate that a vehicle has a new
    //! onboard available mission. The type will be described in the key of the MissionItem but should be of type AUTO.
    //! \param sender
    //! \param missionList
    //!
    virtual void EventVehicle_NewOnboardVehicleMission(const void *sender, const MissionItem::MissionList &missionList) = 0;

    //!
    //! \brief EventVehicle_MissionACK
    //! \param sender
    //! \param ack
    //!
    virtual void EventVehicle_MissionACK(const void *sender, const MissionItem::MissionACK &ack) = 0;

    virtual void EventVehicle_REJECTProposedMission(const void *sender, const Data::MissionKey &key) = 0;

    //virtual void EventVehicle_ACKProposedMissionWChanges(const void *sender, const Data::MissionKey &originalKey, const Data::MissionACK &ackCode, const Data::MissionKey &newKey) = 0;
};

} //End MaceCore Namespace

#endif // I_MODULE_VEHICLE_EVENTS_H
