#ifndef ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
#define ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H

#include "abstract_module_event_listeners.h"
#include "metadata_vehicle.h"

#include "data_generic_command_item/command_item_components.h"

#define BASE_MODULE_VEHICLE_LISTENER_ENUMS EMIT_HEARTBEAT, ISSUE_GENERAL_COMMAND, \
    CHANGE_VEHICLE_ARM, REQUEST_VEHICLE_TAKEOFF, REQUEST_VEHICLE_LAND, REQUEST_VEHICLE_RTL, CHANGE_VEHICLE_MODE, \
    SET_MISSION_STATE, REQUEST_DATA_SYNC, \
    UPLOAD_MISSION, SET_CURRENT_MISSION, REQUEST_CURRENT_MISSION, REQUEST_MISSION, CLEAR_CURRENT_MISSION, \
    REQUEST_ONBOARD_AUTO_MISSION, CLEAR_ONBOARD_AUTO_MISSION, \
    REQUEST_ONBOARD_GUIDED_MISSION, CLEAR_ONBOARD_GUIDED_MISSION, \
    REQUEST_VEHICLE_HOME, SET_VEHICLE_HOME, \
    FOLLOW_NEW_COMMANDS,FINISH_AND_FOLLOW_COMMANDS,COMMANDS_APPENDED

namespace MaceCore
{

class MaceCore;

//!
//! \brief A abstract class that will set up nessessary methods to consume vehicle states
//!
template<typename T, typename I, typename CT>
class AbstractModule_VehicleListener : public AbstractModule_EventListeners<T, I, CT>
{
friend class MaceCore;

public:

    AbstractModule_VehicleListener() :
        AbstractModule_EventListeners<T,I, CT>()
    {
        //These are from MACE Core to modules

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
        /// command and action sequence that accompanies the vheicle. Expect an acknowledgement
        /// or an event to take place when calling these items.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_DATA_SYNC, [this](const int &targetSystem, const OptionalParameter<ModuleCharacteristic> &sender){
            Request_FullDataSync(targetSystem, sender);
        });

        this->template AddCommandLogic<CommandItem::ActionArm>(CT::CHANGE_VEHICLE_ARM, [this](const CommandItem::ActionArm &command, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_SystemArm(command, sender);
        });

        this->template AddCommandLogic<CommandItem::SpatialTakeoff>(CT::REQUEST_VEHICLE_TAKEOFF, [this](const CommandItem::SpatialTakeoff &command, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_VehicleTakeoff(command, sender);
        });

        this->template AddCommandLogic<CommandItem::SpatialLand>(CT::REQUEST_VEHICLE_LAND, [this](const CommandItem::SpatialLand &command, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_Land(command, sender);
        });

        this->template AddCommandLogic<CommandItem::SpatialRTL>(CT::REQUEST_VEHICLE_RTL, [this](const CommandItem::SpatialRTL &command, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_ReturnToLaunch(command, sender);
        });

        this->template AddCommandLogic<CommandItem::ActionMissionCommand>(CT::SET_MISSION_STATE, [this](const CommandItem::ActionMissionCommand &command, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_MissionState(command, sender);
        });

        this->template AddCommandLogic<std::shared_ptr<CommandItem::AbstractCommandItem>>(CT::ISSUE_GENERAL_COMMAND, [this](const std::shared_ptr<CommandItem::AbstractCommandItem> &command, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            Command_IssueGeneralCommand(command);
        });


        this->template AddCommandLogic<CommandItem::ActionChangeMode>(CT::CHANGE_VEHICLE_MODE, [this](const CommandItem::ActionChangeMode &command, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_ChangeSystemMode(command, sender);
        });




        //////////////////////////////////////////////////////////////////////////////////////////////
        /// GENERAL MISSION EVENTS: This functionality may be pertinent for vehicles not containing a
        /// direct MACE hardware module.
        /////////////////////////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<MissionItem::MissionList>(CT::UPLOAD_MISSION, [this](const MissionItem::MissionList &missionList, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            Command_UploadMission(missionList);
        });

        this->template AddCommandLogic<MissionItem::MissionKey>(CT::SET_CURRENT_MISSION, [this](const MissionItem::MissionKey &key, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            Command_SetCurrentMission(key);
        });

        this->template AddCommandLogic<int>(CT::REQUEST_CURRENT_MISSION, [this](const int &targetSystem, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            Command_GetCurrentMission(targetSystem);
        });

        this->template AddCommandLogic<MissionItem::MissionKey>(CT::REQUEST_MISSION, [this](const MissionItem::MissionKey &key, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_GetMission(key, sender);
        });

        this->template AddCommandLogic<int>(CT::CLEAR_CURRENT_MISSION, [this](const int &targetSystem, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            Command_ClearCurrentMission(targetSystem);
        });

        ////////////////////////////////////////////////////////////////////////////
        /// GENERAL AUTO EVENTS: This is implying for auto mode of the vehicle.
        /// This functionality is pertinent for vehicles that may contain a
        /// MACE HW module, or, vehicles that have timely or ever updating changes.
        ////////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_ONBOARD_AUTO_MISSION, [this](const int &targetSystem, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            Command_GetOnboardAuto(targetSystem);
        });

        this->template AddCommandLogic<int>(CT::CLEAR_ONBOARD_AUTO_MISSION, [this](const int &targetSystem, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            Command_ClearOnboardAuto(targetSystem);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
        /// This functionality is pertinent for vehicles that may contain a
        /// MACE HW module, or, vehicles that have timely or ever updating changes.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_ONBOARD_GUIDED_MISSION, [this](const int &targetSystem, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            Command_GetOnboardGuided(targetSystem);
        });

        this->template AddCommandLogic<int>(CT::CLEAR_ONBOARD_GUIDED_MISSION, [this](const int &targetSystem, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            Command_ClearOnboardGuided(targetSystem);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL HOME EVENTS: These events are related to establishing or setting
        /// a home position. It should be recognized that the first mission item in a
        /// mission queue should prepend this position. Just the way ardupilot works.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_VEHICLE_HOME, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_GetHomePosition(vehicleID, sender);
        });

        this->template AddCommandLogic<CommandItem::SpatialHome>(CT::SET_VEHICLE_HOME, [this](const CommandItem::SpatialHome &vehicleHome, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_SetHomePosition(vehicleHome, sender);
        });

    }

public:

    virtual void Request_FullDataSync(const int &targetSystem, const OptionalParameter<ModuleCharacteristic>& = OptionalParameter<ModuleCharacteristic>()) = 0;

    virtual void Command_SystemArm(const CommandItem::ActionArm &command, const OptionalParameter<ModuleCharacteristic> &sender) = 0;
    virtual void Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &command, const OptionalParameter<ModuleCharacteristic> &sender) = 0;
    virtual void Command_Land(const CommandItem::SpatialLand &command, const OptionalParameter<ModuleCharacteristic> &sender) = 0;
    virtual void Command_ReturnToLaunch(const CommandItem::SpatialRTL &command, const OptionalParameter<ModuleCharacteristic> &sender) = 0;
    virtual void Command_MissionState(const CommandItem::ActionMissionCommand &command, const OptionalParameter<ModuleCharacteristic> &sender) = 0;
    virtual void Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command) = 0;

    virtual void Command_ChangeSystemMode(const CommandItem::ActionChangeMode &vehicleMode, const OptionalParameter<ModuleCharacteristic> &sender) = 0;

    virtual void Command_UploadMission(const MissionItem::MissionList &missionList) = 0;
    virtual void Command_SetCurrentMission(const MissionItem::MissionKey &key) = 0;
    virtual void Command_GetCurrentMission(const int &targetSystem) = 0;
    virtual void Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<ModuleCharacteristic>& = OptionalParameter<ModuleCharacteristic>()) = 0;
    virtual void Command_ClearCurrentMission(const int &targetSystem) = 0;

    virtual void Command_GetOnboardAuto(const int &targetSystem) = 0;
    virtual void Command_ClearOnboardAuto(const int &targetSystem) = 0;

    virtual void Command_GetOnboardGuided(const int &targetSystem) = 0;
    virtual void Command_ClearOnboardGuided(const int &targetSystem) = 0;

    virtual void Command_GetHomePosition(const int &vehicleID, const OptionalParameter<ModuleCharacteristic>& = OptionalParameter<ModuleCharacteristic>()) = 0;
    virtual void Command_SetHomePosition(const CommandItem::SpatialHome &vehicleHome, const OptionalParameter<ModuleCharacteristic>& = OptionalParameter<ModuleCharacteristic>()) = 0;


};

} //end of namespace MaceCore

#endif // ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
