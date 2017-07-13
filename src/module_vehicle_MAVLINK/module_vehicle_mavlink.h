#ifndef MODULE_VEHICLE_MAVLINK_H
#define MODULE_VEHICLE_MAVLINK_H

#include "module_vehicle_mavlink_global.h"

#include <iostream>
#include <QMap>
#include <QThread>
#include <QSerialPort>

#include "common/common.h"

#include "module_vehicle_generic/module_vehicle_generic.h"

#include "comms/comms_marshaler.h"
#include "comms/i_protocol_mavlink_events.h"
#include "comms/serial_configuration.h"

#include "comms/serial_link.h"
#include "comms/udp_link.h"
#include "comms/protocol_mavlink.h"

#include "data_vehicle_MAVLINK/mavlink_parser.h"

#include "mace_core/module_factory.h"

#include "data_vehicle_MAVLINK/components.h"
#include "data_vehicle_MAVLINK/vehicle_object_mavlink.h"

#include "commsMAVLINK/comms_mavlink.h"
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
 * Each module will implement commands as defined by it's interface.
 * These commands will NOT be invoked on the thread the module is operating on.
 * If the command is to kick off some action on the module's thread, it will have to marshaled onto the event loop in some way.
 *
 * */

template <typename ...VehicleTopicAdditionalComponents>
class MODULE_VEHICLE_MAVLINKSHARED_EXPORT ModuleVehicleMAVLINK :
        public ModuleVehicleGeneric<VehicleTopicAdditionalComponents..., DATA_VEHICLE_MAVLINK_TYPES>,
        public CommsMAVLINK
{
protected:
    typedef ModuleVehicleGeneric<VehicleTopicAdditionalComponents..., DATA_VEHICLE_MAVLINK_TYPES> ModuleVehicleMavlinkBase;

public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///             CONFIGURE
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ModuleVehicleMAVLINK() :
        ModuleVehicleGeneric<VehicleTopicAdditionalComponents..., DataMAVLINK::EmptyMAVLINK>(),
        airborneInstance(false)
    {

    }


    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const
    {

        MaceCore::ModuleParameterStructure structure;
        ConfigureMAVLINKStructure(structure);

        std::shared_ptr<MaceCore::ModuleParameterStructure> moduleSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
        moduleSettings->AddTerminalParameters("AirborneInstance", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
        structure.AddNonTerminal("ModuleParameters", moduleSettings, true);

        return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
    }


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
    {
        ConfigureComms(params);
        if(params->HasNonTerminal("ModuleParameters"))
        {
            std::shared_ptr<MaceCore::ModuleParameterValue> moduleSettings = params->GetNonTerminalValue("ModuleParameters");
            airborneInstance = moduleSettings->GetTerminalValue<bool>("AirborneInstance");
        }
    }

   public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///              MACE COMMANDS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    virtual void Command_SystemArm(const CommandItem::ActionArm &vehicleArm)
    {
        UNUSED(vehicleArm);
    }

    virtual void Command_ChangeSystemMode(const CommandItem::ActionChangeMode &vehicleMode)
    {
        UNUSED(vehicleMode);
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///              COMM EVENTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual void VehicleHeartbeatInfo(const std::string &linkName, const int systemID, const mavlink_heartbeat_t &heartbeatMSG)
    {
        UNUSED(linkName);
        UNUSED(systemID);
        UNUSED(heartbeatMSG);
    }

    virtual void MAVLINKCommandAck(const std::string &linkName, const int systemID, const mavlink_command_ack_t &cmdACK)
    {
        UNUSED(linkName);
        UNUSED(systemID);
        UNUSED(cmdACK);
    }

    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
    {
        UNUSED(linkName);
        UNUSED(message);
    }


    //!
    //! \brief New heartbeat from MAVLINK received over a link
    //! \param linkName Name of link
    //! \param vehicleId
    //! \param vehicleMavlinkVersion
    //! \param vehicleFirmwareType
    //! \param vehicleType
    //!
    virtual void VehicleHeartbeatInfo(const std::string &linkName, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const
    {
        UNUSED(linkName);
        UNUSED(vehicleId);
        UNUSED(vehicleMavlinkVersion);
        UNUSED(vehicleFirmwareType);
        UNUSED(vehicleType);
        //incomming heartbeats
    }

    virtual void RequestDummyFunction(const int &vehicleID)
    {
        UNUSED(vehicleID);
    }

protected:
    bool airborneInstance;

private:
    std::map<int,DataMAVLINK::VehicleObject_MAVLINK*> m_MAVLINKData;

};

#endif // MODULE_VEHICLE_MAVLINK_H
