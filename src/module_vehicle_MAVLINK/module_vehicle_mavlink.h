#ifndef MODULE_VEHICLE_MAVLINK_H
#define MODULE_VEHICLE_MAVLINK_H

#include "common/common.h"

#include "module_vehicle_mavlink_global.h"

#include "mace_core/i_module_command_vehicle.h"

#include "comms/link_marshaler.h"
#include "comms/i_mavlink_protocol_events.h"

#include "comms/serial_configuration.h"


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

class MODULE_VEHICLE_MAVLINKSHARED_EXPORT ModuleVehicleMAVLINK : public MaceCore::IModuleCommandVehicle, public Comms::IMavlinkCommsEvents
{

public:

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
    //! Method will be called on module's thread
    //!
    //!
    virtual void FollowNewCommands();


    //!
    //! \brief New commands have been issued to vehicle that are to be followed once current command is finished
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //! Method will be called on module's thread
    //!
    virtual void FinishAndFollowNewCommands();


    //!
    //! \brief New commands have been appended to existing commands
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //! Method will be called on module's thread
    //!
    virtual void CommandsAppended();


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///              PROTOCOL EVENTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////


    //!
    //! \brief A message about protocol has been generated
    //! \param title
    //! \param message
    //!
    virtual void ProtocolStatusMessage(const std::string &title, const std::string &message) const;

    //!
    //! \brief A Message has been received over Mavlink protocol
    //! \param message Message that has been received
    //!
    virtual void MessageReceived(const mavlink_message_t &message) const;

    //!
    //! \brief Heartbeat of vehicle received
    //! \param link
    //! \param vehicleId
    //! \param vehicleMavlinkVersion
    //! \param vehicleFirmwareType
    //! \param vehicleType
    //!
    virtual void VehicleHeartbeatInfo(const std::string &linkName, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const;

    virtual void ReceiveLossPercentChanged(int uasId, float lossPercent) const;
    virtual void ReceiveLossTotalChanged(int uasId, int totalLoss) const;


    //!
    //! \brief A new radio status packet received
    //! \param link
    //! \param rxerrors
    //! \param fixed
    //! \param rssi
    //! \param remrssi
    //! \param txbuf
    //! \param noise
    //! \param remnoise
    //!
    virtual void RadioStatusChanged(const std::string &linkName, unsigned rxerrors, unsigned fixed, int rssi, int remrssi, unsigned txbuf, unsigned noise, unsigned remnoise) const;

private:

    Comms::LinkMarshaler *m_LinkMarshler;

    std::unordered_map<Comms::Protocols, std::shared_ptr<Comms::ProtocolConfiguration>, EnumClassHash> m_AvailableProtocols;

};

#endif // MODULE_VEHICLE_MAVLINK_H
