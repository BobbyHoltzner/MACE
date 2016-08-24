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
 *
 * The start method is the entry point for the thread that the module is to run on.
 * The start() method should contain an event loop of some sort that responds to commands made.
 *
 * Each module will impliment commands as defined by it's interface.
 * These commands will NOT be invoked on the thread the module is operating on.
 * If the command is to kick off some action on the module's thread, it will have to marshaled onto the event loop in some way.
 *
 * */

class MODULE_VEHICLE_MAVLINKSHARED_EXPORT ModuleVehicleMAVLINK : public MaceCore::IModuleCommandVehicle
{

public:
    ModuleVehicleMAVLINK(const MaceCore::MetadataVehicle &vehicleMetaData);


    //!
    //! \brief function that is to kick off the Vehicle Comms event loop
    //!
    virtual void start();


public:


    //!
    //! \brief New commands have been updated that the vehicle is to follow immediatly
    //!
    virtual void FollowNewCommands();


    //!
    //! \brief New commands have been issued to vehicle that are to be followed once current command is finished
    //!
    virtual void FinishAndFollowNewCommands();


    //!
    //! \brief New commands have been appended to existing commands
    //!
    virtual void CommandsAppended();

};

#endif // MODULE_VEHICLE_MAVLINK_H
