#ifndef MODULE_VEHICLE_ARDUPILOT_H
#define MODULE_VEHICLE_ARDUPILOT_H

#include <map>

#include "spdlog/spdlog.h"

#include <mavlink.h>
#include "data/mission_command.h"

#include "ardupilot_guided_controller.h"
#include "ardupilot_takeoff_controller.h"

#include "module_vehicle_ardupilot_global.h"
#include "module_vehicle_MAVLINK/module_vehicle_mavlink.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_state_item/state_item_components.h"
#include "data_generic_command_item/command_item_components.h"

#include "data_generic_state_item_topic/state_topic_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "ardupilot_states/ardupilot_hsm.h"
#include "ardupilot_states/state_components.h"
#include "vehicle_object/ardupilot_vehicle_object.h"

//__________________
#include "data_interface_MAVLINK/callback_interface_data_mavlink.h"

#include "data_interface_MAVLINK/vehicle_object_mavlink.h"

#include "base_topic/vehicle_topics.h"

#include "controllers/I_controller.h"
#include "controllers/I_message_notifier.h"

#include "module_vehicle_MAVLINK/controllers/controller_mission.h"

using namespace std::placeholders;

//class MODULE_VEHICLE_ARDUPILOTSHARED_EXPORT ModuleVehicleArdupilot : public ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>, public DataInterface_MAVLINK::CallbackInterface_DataMAVLINK
class MODULE_VEHICLE_ARDUPILOTSHARED_EXPORT ModuleVehicleArdupilot : public ModuleVehicleMAVLINK<>
{
public:
    ModuleVehicleArdupilot();

    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    void createLog(const int &systemID);

    void MissionAcknowledgement(const MAV_MISSION_RESULT &missionResult, const bool &publishResult);

private:

    void SpinUpController(Ardupilot_GeneralController *newController);

    void SpinDownController();

public:

    //callback interface support for the DataInterface_MAVLINK object
    void cbi_VehicleCommandACK(const int &systemID, const mavlink_command_ack_t &cmdACK);
    void cbi_VehicleMissionACK(const MissionItem::MissionACK &ack);

    virtual void VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG);

    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual bool MavlinkMessage(const std::string &linkName, const mavlink_message_t &msg);


    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    virtual void NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target);


    //!
    //! \brief New Spooled topic given
    //!
    //! Spooled topics are stored on the core's datafusion.
    //! This method is used to notify other modules that there exists new data for the given components on the given module.
    //! \param topicName Name of topic given
    //! \param sender Module that sent topic
    //! \param componentsUpdated Components in topic that where updated
    //! \param target Target moudle (or broadcast)
    //!
    virtual void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>());


    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr);


    //!
    //! \brief PublishVechicleData
    //! \param components
    //!
    void PublishVehicleData(const int &systemID, const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &components);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from IModuleCommandVehicle via AbstractModuleBaseVehicleListener.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:

    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
    /// command and action sequence that accompanies the vheicle. Expect an
    /// acknowledgement or an event to take place when calling these items.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Request_FullDataSync
    //! \param targetSystem
    //!
    virtual void Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic>& = OptionalParameter<MaceCore::ModuleCharacteristic>());

    //!
    //! \brief Command_SystemArm
    //! \param vehicleArm
    //!
    virtual void Command_SystemArm(const CommandItem::ActionArm &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_VehicleTakeoff
    //! \param vehicleTakeoff
    //!
    virtual void Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_Land
    //! \param command
    //!
    virtual void Command_Land(const CommandItem::SpatialLand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_ReturnToLaunch
    //! \param command
    //!
    virtual void Command_ReturnToLaunch(const CommandItem::SpatialRTL &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_MissionState
    //! \param command
    //!
    virtual void Command_MissionState(const CommandItem::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_ChangeSystemMode
    //! \param vehicleMode
    //!
    virtual void Command_ChangeSystemMode(const CommandItem::ActionChangeMode &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_IssueGeneralCommand
    //! \param command
    //!
    virtual void Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command);

    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL MISSION EVENTS:
    ////////////////////////////////////////////////////////////////////////////

    virtual void Command_UploadMission(const MissionItem::MissionList &missionList);

    virtual void Command_SetCurrentMission(const MissionItem::MissionKey &key);

    virtual void Command_GetCurrentMission(const int &targetSystem);

    virtual void Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>());

    virtual void Command_ClearCurrentMission(const int &targetSystem);

    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL AUTO EVENTS: This is implying for auto mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    ////////////////////////////////////////////////////////////////////////////

    virtual void Command_GetOnboardAuto(const int &targetSystem);

    virtual void Command_ClearOnboardAuto(const int &targetSystem);

    /////////////////////////////////////////////////////////////////////////
    /// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    /////////////////////////////////////////////////////////////////////////

    virtual void Command_GetOnboardGuided(const int &targetSystem);

    virtual void Command_ClearOnboardGuided(const int &targetSystem);


    //THE OLD ONES AND THEN WE COMPARE
    /////////////////////////////////////////////////////////////////////////
    /// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
    /// This functionality may be pertinent for vehicles not containing a
    /// direct MACE hardware module.
    /////////////////////////////////////////////////////////////////////////


    //!
    //! \brief UpdateMissionKey
    //! \param key
    //!
    virtual void UpdateMissionKey(const MissionItem::MissionKeyChange &key);


    /////////////////////////////////////////////////////////////////////////////
    /// GENERAL HOME EVENTS: These events are related to establishing or setting
    /// a home position. It should be recognized that the first mission item in a
    /// mission queue should prepend this position. Just the way ardupilot works.
    /////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetHomePosition
    //! \param vehicleID
    //!
    virtual void Command_GetHomePosition (const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic>& = OptionalParameter<MaceCore::ModuleCharacteristic>());

    //!
    //! \brief Command_SetHomePosition
    //! \param vehicleHome
    //!
    virtual void Command_SetHomePosition(const CommandItem::SpatialHome &vehicleHome, const OptionalParameter<MaceCore::ModuleCharacteristic>& = OptionalParameter<MaceCore::ModuleCharacteristic>());


    bool checkControllerState()
    {
        if(m_AircraftController)
        {
          //The current controller is not null
            if(m_AircraftController->isThreadActive())
            {
                //The controller is valid and is actively doing something
                return true;
            }
            else{
                //The controller is valid however it is done for some reason
                //The thread is no longer active
                //KEN TODO: We should figure out if this is the proper way to clean this up
                delete m_AircraftController;
                return false;
            }
        }
        return false;
    }

    virtual void RequestDummyFunction(const int &vehicleID)
    {
//        std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
//        Ardupilot_TakeoffController* newController = new Ardupilot_TakeoffController(tmpData, m_LinkMarshaler, m_LinkName, m_LinkChan, std::bind(&ModuleVehicleArdupilot::takeoffCallback, this, _1));
//        CommandItem::SpatialTakeoff takeoff;
//        takeoff.setTargetSystem(1);
//        takeoff.position.setPosition3D(37,-76,100);
//        newController->initializeTakeoffSequence(takeoff);

//        m_AircraftController = newController;
//        std::thread *thread = new std::thread([newController]()
//        {
//            newController->start();
//        });
    }


    //callback stuff temp
private:
    static void staticCallbackFunction_VehicleTarget(void *p, MissionTopic::VehicleTargetTopic &target)
    {
        ((ModuleVehicleArdupilot *)p)->callbackFunction_VehicleTarget(target);
    }

    void callbackFunction_VehicleTarget(const MissionTopic::VehicleTargetTopic &target)
    {
        std::cout << target << std::endl;
        std::shared_ptr<MissionTopic::VehicleTargetTopic> ptrTarget = std::make_shared<MissionTopic::VehicleTargetTopic>(target);
        ModuleVehicleMAVLINK::cbi_VehicleMissionData(target.getVehicleID(),ptrTarget);
    }

private:
    std::shared_ptr<spdlog::logger> mLogs;

private:
    std::shared_ptr<ArdupilotVehicleObject> vehicleData;

private:

    Ardupilot_GeneralController* m_AircraftController;

    hsm::StateMachine* stateMachine; /**< Member variable containing a pointer to the state
 machine. This state machine evolves the state per event updates and/or external commands. */

    std::unordered_map<std::string, Controllers::IController<mavlink_message_t>*> m_TopicToControllers;

    MAVLINKVehicleControllers::ControllerMission * m_MissionController;
};

#endif // MODULE_VEHICLE_ARDUPILOT_H
