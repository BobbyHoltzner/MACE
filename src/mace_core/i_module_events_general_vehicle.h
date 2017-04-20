#ifndef I_MODULE_EVENTS_GENERAL_VEHICLE_H
#define I_MODULE_EVENTS_GENERAL_VEHICLE_H

#include "topic.h"


#include "data/mission_key.h"
#include "data_generic_state_item/state_item_components.h"
#include "data_generic_mission_item/mission_item_components.h"

//A vehicle module can indicate something has happened

namespace MaceCore
{

class IModuleEventsGeneralVehicle
{
public:

    //!
    //! \brief NewConstructedVehicle
    //! \param sender
    //! \param newVehicleObserved
    //!
    virtual void NewConstructedVehicle(const void *sender, const int &newVehicleObserved) = 0;

    //!
    //! \brief NewVehicleHomePosition This function is emitted to alert the core that a module connected to a vehicle
    //! has received and set a new home position for the system. This should typically be in response to a SetVehicleHomePosition
    //! request.
    //! \param sender
    //! \param vehicleHome
    //!
    virtual void NewVehicleHomePosition(const void *sender, const MissionItem::SpatialHome &vehicleHome) = 0;

    //!
    //! \brief NewOnboardVehicleMission
    //! \param sender
    //! \param missionList
    //!
    virtual void NewOnboardVehicleMission(const void *sender, const MissionItem::MissionList &missionList) = 0;

    //!
    //! \brief ConfirmedOnboardVehicleMission
    //! \param sender
    //! \param missionKey
    //!
    virtual void ConfirmedOnboardVehicleMission(const void *sender, const Data::MissionKey &missionKey) = 0;

    //!
    //! \brief UpdateCurrentVehicleMission
    //! \param sender
    //! \param systemID
    //! \param missionID
    //!
    virtual void NewCurrentVehicleMission(const void *sender, const Data::MissionKey &missionKey) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_GENERAL_VEHICLE_H
