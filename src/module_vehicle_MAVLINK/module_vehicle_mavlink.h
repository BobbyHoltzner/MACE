#ifndef MODULE_VEHICLE_MAVLINK_H
#define MODULE_VEHICLE_MAVLINK_H

#include <QMap>

#include "common/common.h"

#include "module_vehicle_mavlink_global.h"

#include "mace_core/i_module_command_vehicle.h"

#include "comms/comms_marshaler.h"
#include "comms/i_protocol_mavlink_events.h"

#include "comms/serial_configuration.h"

#include "data_vehicle/vehicle_state_data.h"
#include "data_vehicle/arducopter_main.h"
#include "data_vehicle/arducopter_collection.h"

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

class MODULE_VEHICLE_MAVLINKSHARED_EXPORT ModuleVehicleMAVLINK : public MaceCore::IModuleCommandVehicle, public Comms::CommsEvents
{

public:
    void gotInfoTest(const Data::VehicleStateData &messageData);
    void gotArducopterMessage(const Data::ArducopterData &messageArducopter);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///             CONFIGURE
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ModuleVehicleMAVLINK();
    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);


public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///              MACE COMMANDS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief New commands have been updated that the vehicle is to follow immediatly
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //!
    //!
    virtual void FollowNewCommands();


    //!
    //! \brief New commands have been issued to vehicle that are to be followed once current command is finished
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //!
    virtual void FinishAndFollowNewCommands();


    //!
    //! \brief New commands have been appended to existing commands
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //!
    virtual void CommandsAppended();


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///              COMM EVENTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////


    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &msg) const;


    //!
    //! \brief New heartbeat from MAVLINK received over a link
    //! \param linkName Name of link
    //! \param vehicleId
    //! \param vehicleMavlinkVersion
    //! \param vehicleFirmwareType
    //! \param vehicleType
    //!
    virtual void VehicleHeartbeatInfo(const std::string &linkName, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const;

private:
    Comms::CommsMarshaler *m_LinkMarshler;

    std::unordered_map<Comms::Protocols, std::shared_ptr<Comms::ProtocolConfiguration>, EnumClassHash> m_AvailableProtocols;

};

#endif // MODULE_VEHICLE_MAVLINK_H
