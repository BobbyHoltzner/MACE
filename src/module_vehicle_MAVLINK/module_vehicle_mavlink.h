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

#include "mace_core/module_factory.h"

#include "data_interface_MAVLINK/components/data_interface_mavlink_components.h"

#include "commsMAVLINK/comms_mavlink.h"

#include "controllers/generic_controller.h"

#include "module_vehicle_MAVLINK/controllers/commands/command_land.h"
#include "module_vehicle_MAVLINK/controllers/commands/command_takeoff.h"
#include "module_vehicle_MAVLINK/controllers/commands/command_arm.h"
#include "module_vehicle_MAVLINK/controllers/commands/command_rtl.h"
#include "module_vehicle_MAVLINK/controllers/controller_system_mode.h"

#include "base_topic/vehicle_topics.h"
#include "data_generic_state_item_topic/state_topic_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"
#include "vehicle_object/mavlink_vehicle_object.h"

#include "mavlink_entity_key.h"

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
        public CommsMAVLINK,
        public CallbackInterface_MAVLINKVehicleObject
{
protected:
    typedef ModuleVehicleGeneric<VehicleTopicAdditionalComponents..., DATA_VEHICLE_MAVLINK_TYPES> ModuleVehicleMavlinkBase;

public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///             CONFIGURE
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ModuleVehicleMAVLINK():
        ModuleVehicleGeneric<VehicleTopicAdditionalComponents..., DataMAVLINK::EmptyMAVLINK>(),
        airborneInstance(false), m_VehicleMissionTopic("vehicleMission"), m_IsAttachedMavlinkEntitySet(false)
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

    virtual void start()
    {
        ModuleVehicleMavlinkBase::start();
    }

   public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///              MACE COMMANDS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_SystemArm Command an ARM/DISARM action
    //! \param vehicleArm Arm action
    //!
    virtual void Command_SystemArm(const CommandItem::ActionArm &vehicleArm)
    {
        UNUSED(vehicleArm);
    }

    //!
    //!= \brief Command_ChangeSystemMode Command a CHANGE MODE action
    //! \param vehicleMode Mode to change to
    //!
    virtual void Command_ChangeSystemMode(const CommandItem::ActionChangeMode &vehicleMode)
    {
        UNUSED(vehicleMode);
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///              COMM EVENTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief VehicleHeartbeatInfo Heartbeat message from vehicle
    //! \param linkName Comms link name
    //! \param systemID Vehicle ID generating heartbeat
    //! \param heartbeatMSG Heartbeat message
    //!
    virtual void VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG)
    {
        UNUSED(linkName);
        UNUSED(systemID);
        UNUSED(heartbeatMSG);
    }

    //!
    //! \brief MAVLINKCommandAck Acknowledge a MAVLINK command
    //! \param linkName Comms link name
    //! \param systemID Vehicle ID generating the ACK
    //! \param cmdACK ACK message
    //!
    virtual void MAVLINKCommandAck(const std::string &linkName, const int systemID, const mavlink_command_ack_t &cmdACK)
    {
        UNUSED(linkName);
        UNUSED(systemID);
        UNUSED(cmdACK);
    }


    //!
    //! \brief MavlinkMessage
    //! \param linkName
    //! \param message
    //! \return if the data was consumed by a controller object
    //!
    virtual bool MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
    {
        UNUSED(linkName);
        UNUSED(message);
        return false;
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

    virtual void UpdateDynamicMissionQueue(const TargetItem::DynamicMissionQueue &queue)
    {
        UNUSED(queue);
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///Functional Interface required from CallbackInterface_MAVLINKVehicleObject
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief cbi_VehicleStateData Callback tied to Vehicle State Data updates
    //! \param systemID Vehicle ID generating the state data
    //! \param data State data
    //!
    virtual void cbi_VehicleStateData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data)
    {
        MaceCore::TopicDatagram topicDatagram;
        this->m_VehicleDataTopic.SetComponent(data, topicDatagram);
        ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, this->m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
    }

    //!
    //! \brief cbi_VehicleMissionData Callback tied to Vehicle Mission data updates
    //! \param systemID Vehicle ID generating the mission data
    //! \param data Mission data
    //!
    virtual void cbi_VehicleMissionData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data) const
    {
        MaceCore::TopicDatagram topicDatagram;
        m_VehicleMissionTopic.SetComponent(data, topicDatagram);
        ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleMissionTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
    }

    //!
    //! \brief cbi_VehicleHome Callback tied to Vehicle Home data
    //! \param systemID Vehicle ID generating the vehicle home data
    //! \param home Vehicle home position data
    //!
    virtual void cbi_VehicleHome(const int &systemID, const CommandItem::SpatialHome &home)
    {
    //    std::stringstream buffer;
    //    buffer << home;

    //    mLogs->debug("Receieved a new vehicle home position.");
    //    mLogs->info(buffer.str());


        //notify the core of the change
        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->GVEvents_NewHomePosition(this, home);
        });

        std::shared_ptr<CommandItem::SpatialHome> ptrHome = std::make_shared<CommandItem::SpatialHome>(home);
        std::shared_ptr<MissionTopic::MissionHomeTopic> homeTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
        homeTopic->setHome(ptrHome);

        this->cbi_VehicleMissionData(systemID,homeTopic);
    }

    //!
    //! \brief cbi_VehicleMission Callback tied to vehicle mission updates
    //! \param systemID Vehicle ID generating the mission
    //! \param missionList Mission list
    //!
    virtual void cbi_VehicleMission(const int &systemID, const MissionItem::MissionList &missionList)
    {
    //    std::stringstream buffer;
    //    buffer << missionList;

    //    mLogs->info("Receieved a new vehicle mission.");
    //    mLogs->info(buffer.str());

        //This function shall update the local MACE CORE instance of the mission
        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->EventVehicle_NewOnboardVehicleMission(this, missionList);
        });
        //We should update all listeners
        std::shared_ptr<MissionTopic::MissionListTopic> missionTopic = std::make_shared<MissionTopic::MissionListTopic>(missionList);

        //This function shall update the local vehicle object with the current vehicle mission
        //vehicleData->mission->setCurrentMission(missionList);
        this->cbi_VehicleMissionData(systemID,missionTopic);
    }

    //!
    //! \brief cbi_VehicleMissionItemCurrent Callback tied to current mission item updates
    //! \param current Current mission item
    //!
    virtual void cbi_VehicleMissionItemCurrent(const MissionItem::MissionItemCurrent &current) const
    {
    //    std::stringstream buffer;
    //    buffer << current.getMissionKey();

    //    mLogs->info("The vehicle module has seen a current mission item" + std::to_string(current.getMissionCurrentIndex()));
    //    mLogs->debug(buffer.str());

        //This function shall update the local MACE core of the new mission
        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->GVEvents_MissionItemCurrent(this, current);
        });

        std::shared_ptr<MissionTopic::MissionItemCurrentTopic> ptrMissionTopic = std::make_shared<MissionTopic::MissionItemCurrentTopic>(current);
        cbi_VehicleMissionData(current.getMissionKey().m_systemID,ptrMissionTopic);
    }


public:

    void SetAttachedMavlinkEntity(const MavlinkEntityKey &key)
    {
        m_AttachedMavlinkEntity = key;
        m_IsAttachedMavlinkEntitySet = true;
    }

    MavlinkEntityKey GetAttachedMavlinkEntity() const
    {
        if(m_IsAttachedMavlinkEntitySet == false)
        {
            throw std::runtime_error("Contained MAVLINK entitiy has yet to be set!");
        }
        return m_AttachedMavlinkEntity;
    }

private:

    MavlinkEntityKey m_AttachedMavlinkEntity;
    bool m_IsAttachedMavlinkEntitySet;

protected:
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_VehicleMissionTopic;

    BaseTopic::VehicleTopics m_VehicleTopics;

protected:
    //!
    //! \brief airborneInstance Flag denoting airborne or ground instance
    //!
    bool airborneInstance;

//    PointerCollection<
//        Controllers::ControllerHome<mavlink_message_t>,
//        Controllers::ControllerMission<mavlink_message_t>
//    > m_Controllers;


};

#endif // MODULE_VEHICLE_MAVLINK_H
