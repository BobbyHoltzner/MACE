#ifndef MODULE_VEHICLE_MAVLINK_H
#define MODULE_VEHICLE_MAVLINK_H

#include "module_vehicle_mavlink_global.h"

#include "mace_core/i_module_command_vehicle.h"




/*
 *
 * USAGE:
 *
 * Insert the nessessary code to do Vehicle communications with MAVLINK
 *
 * Look at i_module_events_vehicle.h in mace_core for the events that can be triggered.
 * Feel free to add any nessessary events (if an event is added, its handler must also be added in mace_core.h
 *
 * When it comes time to signal an event to MaceCore do so by calling the following code structure:
 *      NotifyListeners([&](IModuleEventsVehicle *obj){obj->NewPositionDynamics(this, arg1, arg2, ... , argN);});
 * Replacing "NewPositionDynamics" with the event of your choice, and replacing arguments with what is required for that event
 * */

class MODULE_VEHICLE_MAVLINKSHARED_EXPORT ModuleVehicleMAVLINK : public MaceCore::IModuleCommandVehicle
{

public:
    ModuleVehicleMAVLINK(const MaceCore::MetadataVehicle &vehicleMetaData);


    //!
    //! \brief Issue a new target position for the vehicle
    //! \param position Position for the vehicle to achieve
    //!
    virtual void IssueTarget(const Eigen::Vector3d &position);

};

#endif // MODULE_VEHICLE_MAVLINK_H
