#ifndef I_MODULE_EVENTS_GENERAL_VEHICLE_H
#define I_MODULE_EVENTS_GENERAL_VEHICLE_H

#include "topic.h"

#include "data/mission_execution_state.h"
#include "data/mission_key.h"
#include "data_generic_state_item/state_item_components.h"
#include "data_generic_command_item/command_item_components.h"

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
    //! \brief GVEvents_NewHomePosition This function is emitted to alert the core that a module connected to a vehicle
    //! has received and set a new home position for the system. This should typically be in response to a Command_SetHomePosition
    //! request.
    //! \param sender
    //! \param vehicleHome
    //!
    virtual void GVEvents_NewHomePosition(const void *sender, const CommandItem::SpatialHome &vehicleHome) = 0;

    //!
    //! \brief GVEvents_MissionExecutionStateUpdated
    //! \param sender
    //! \param missionKey
    //! \param missionExeState
    //!
    virtual void GVEvents_MissionExeStateUpdated(const void *sender, const Data::MissionKey &missionKey, const Data::MissionExecutionState &missionExeState) = 0;

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
