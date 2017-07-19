#ifndef MODULE_VEHICLE_ARDUPILOT_H
#define MODULE_VEHICLE_ARDUPILOT_H

#include <map>

#include <mavlink.h>

#include "data/timer.h"
#include "data/mission_command.h"

#include "ardupilot_guided_controller.h"
#include "ardupilot_takeoff_controller.h"

#include "module_vehicle_ardupilot_global.h"
#include "module_vehicle_MAVLINK/module_vehicle_mavlink.h"

#include "data_vehicle_ardupilot/components.h"
#include "data_vehicle_ardupilot/vehicle_object_ardupilot.h"

#include "data_vehicle_MAVLINK/MACE_to_MAVLINK/command_mace_to_mavlink.h"
#include "data_vehicle_MAVLINK/MACE_to_MAVLINK/mission_mace_to_mavlink.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

//__________________
#include "data_vehicle_MAVLINK/MACE_to_MAVLINK/command_mace_to_mavlink.h"

#include "data_interface_MAVLINK/vehicle_object_mavlink.h"

using namespace std::placeholders;

class MODULE_VEHICLE_ARDUPILOTSHARED_EXPORT ModuleVehicleArdupilot : public ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>
{
enum ArdupilotMissionMode{
    NONE,
    REQUESTING,
    TRANSMITTING
};

public:
    ModuleVehicleArdupilot();

    bool ParseMAVLINKMissionMessage(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, const std::string &linkName, const mavlink_message_t *message);

    void MissionAcknowledgement(const MAV_MISSION_RESULT &missionResult, const bool &publishResult);

private:

    void SpinUpController(Ardupilot_GeneralController *newController);

    void SpinDownController();


public:

    virtual void VehicleHeartbeatInfo(const std::string &linkName, const int systemID, const mavlink_heartbeat_t &heartbeatMSG);

    virtual void MAVLINKCommandAck(const std::string &linkName, const int systemID, const mavlink_command_ack_t &cmdACK);
    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &msg);

    //!
    //! \brief NewTopic
    //! \param topicName
    //! \param senderID
    //! \param componentsUpdated
    //!
    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);

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
    virtual void Request_FullDataSync(const int &targetSystem);

    //!
    //! \brief Command_SystemArm
    //! \param vehicleArm
    //!
    virtual void Command_SystemArm(const CommandItem::ActionArm &command);

    //!
    //! \brief Command_VehicleTakeoff
    //! \param vehicleTakeoff
    //!
    virtual void Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &command);

    //!
    //! \brief Command_Land
    //! \param command
    //!
    virtual void Command_Land(const CommandItem::SpatialLand &command);

    //!
    //! \brief Command_ReturnToLaunch
    //! \param command
    //!
    virtual void Command_ReturnToLaunch(const CommandItem::SpatialRTL &command);

    //!
    //! \brief Command_MissionState
    //! \param command
    //!
    virtual void Command_MissionState(const CommandItem::ActionMissionCommand &command);

    //!
    //! \brief Command_ChangeSystemMode
    //! \param vehicleMode
    //!
    virtual void Command_ChangeSystemMode(const CommandItem::ActionChangeMode &command);

    //!
    //! \brief Command_IssueGeneralCommand
    //! \param command
    //!
    virtual void Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command);

    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL MISSION EVENTS:
    ////////////////////////////////////////////////////////////////////////////

    virtual void Command_UploadMission(const MissionItem::MissionList &missionList);

    virtual void Command_SetCurrentMission(const Data::MissionKey &key);

    virtual void Command_GetCurrentMission(const int &targetSystem);

    virtual void Command_GetMission(const Data::MissionKey &key);

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
    virtual void UpdateMissionKey(const Data::MissionKeyChange &key);


    /////////////////////////////////////////////////////////////////////////////
    /// GENERAL HOME EVENTS: These events are related to establishing or setting
    /// a home position. It should be recognized that the first mission item in a
    /// mission queue should prepend this position. Just the way ardupilot works.
    /////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetHomePosition
    //! \param vehicleID
    //!
    virtual void Command_GetHomePosition (const int &vehicleID);

    //!
    //! \brief Command_SetHomePosition
    //! \param vehicleHome
    //!
    virtual void Command_SetHomePosition(const CommandItem::SpatialHome &vehicleHome);

    //!
    //! \brief homePositionUpdated
    //! \param newVehicleHome
    //!
    void homePositionUpdated(const CommandItem::SpatialHome &newVehicleHome);


    // Controller Callbacks:
    void takeoffCallback(const std::string value);
    void guidedCallback(const std::string value);


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
        std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
        Ardupilot_TakeoffController* newController = new Ardupilot_TakeoffController(tmpData, m_LinkMarshaler, m_LinkName, m_LinkChan, std::bind(&ModuleVehicleArdupilot::takeoffCallback, this, _1));
        CommandItem::SpatialTakeoff takeoff;
        takeoff.setTargetSystem(1);
        takeoff.position.setPosition3D(37,-76,100);
        newController->initializeTakeoffSequence(takeoff);

        m_AircraftController = newController;
        std::thread *thread = new std::thread([newController]()
        {
            newController->start();
        });
    }

private:
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> getArducopterData(const int &systemID);
private:

    Timer t;

private:
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_VehicleMission;

    std::map<int, std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT>> m_ArduPilotData;

    Ardupilot_GeneralController* m_AircraftController;
};

#endif // MODULE_VEHICLE_ARDUPILOT_H
