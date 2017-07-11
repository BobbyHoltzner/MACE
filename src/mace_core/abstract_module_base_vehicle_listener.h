#ifndef ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
#define ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H

#include "abstract_module_event_listeners.h"
#include "metadata_vehicle.h"

#include "data/mission_key.h"

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

        this->template AddCommandLogic<int>(CT::REQUEST_DATA_SYNC, [this](const int &targetSystem){
            Request_FullDataSync(targetSystem);
        });

        this->template AddCommandLogic<CommandItem::ActionArm>(CT::CHANGE_VEHICLE_ARM, [this](const CommandItem::ActionArm &command){
            Command_SystemArm(command);
        });

        this->template AddCommandLogic<CommandItem::SpatialTakeoff>(CT::REQUEST_VEHICLE_TAKEOFF, [this](const CommandItem::SpatialTakeoff &command){
            Command_VehicleTakeoff(command);
        });

        this->template AddCommandLogic<CommandItem::SpatialLand>(CT::REQUEST_VEHICLE_LAND, [this](const CommandItem::SpatialLand &command){
            Command_Land(command);
        });

        this->template AddCommandLogic<CommandItem::SpatialRTL>(CT::REQUEST_VEHICLE_RTL, [this](const CommandItem::SpatialRTL &command){
            Command_ReturnToLaunch(command);
        });

        this->template AddCommandLogic<CommandItem::ActionMissionCommand>(CT::SET_MISSION_STATE, [this](const CommandItem::ActionMissionCommand &command){
            Command_MissionState(command);
        });

        this->template AddCommandLogic<std::shared_ptr<CommandItem::AbstractCommandItem>>(CT::ISSUE_GENERAL_COMMAND, [this](const std::shared_ptr<CommandItem::AbstractCommandItem> &command){
            Command_IssueGeneralCommand(command);
        });


        this->template AddCommandLogic<CommandItem::ActionChangeMode>(CT::CHANGE_VEHICLE_MODE, [this](const CommandItem::ActionChangeMode &command){
            Command_ChangeSystemMode(command);
        });




        //////////////////////////////////////////////////////////////////////////////////////////////
        /// GENERAL MISSION EVENTS: This functionality may be pertinent for vehicles not containing a
        /// direct MACE hardware module.
        /////////////////////////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<MissionItem::MissionList>(CT::UPLOAD_MISSION, [this](const MissionItem::MissionList &missionList){
            Command_UploadMission(missionList);
        });

        this->template AddCommandLogic<Data::MissionKey>(CT::SET_CURRENT_MISSION, [this](const Data::MissionKey &key){
            Command_SetCurrentMission(key);
        });

        this->template AddCommandLogic<int>(CT::REQUEST_CURRENT_MISSION, [this](const int &targetSystem){
            Command_GetCurrentMission(targetSystem);
        });

        this->template AddCommandLogic<Data::MissionKey>(CT::REQUEST_MISSION, [this](const Data::MissionKey &key){
            Command_GetMission(key);
        });

        this->template AddCommandLogic<int>(CT::CLEAR_CURRENT_MISSION, [this](const int &targetSystem){
            Command_ClearCurrentMission(targetSystem);
        });

        ////////////////////////////////////////////////////////////////////////////
        /// GENERAL AUTO EVENTS: This is implying for auto mode of the vehicle.
        /// This functionality is pertinent for vehicles that may contain a
        /// MACE HW module, or, vehicles that have timely or ever updating changes.
        ////////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_ONBOARD_AUTO_MISSION, [this](const int &targetSystem){
            Command_GetOnboardAuto(targetSystem);
        });

        this->template AddCommandLogic<int>(CT::CLEAR_ONBOARD_AUTO_MISSION, [this](const int &targetSystem){
            Command_ClearOnboardAuto(targetSystem);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
        /// This functionality is pertinent for vehicles that may contain a
        /// MACE HW module, or, vehicles that have timely or ever updating changes.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_ONBOARD_GUIDED_MISSION, [this](const int &targetSystem){
            Command_GetOnboardGuided(targetSystem);
        });

        this->template AddCommandLogic<int>(CT::CLEAR_ONBOARD_GUIDED_MISSION, [this](const int &targetSystem){
            Command_ClearOnboardGuided(targetSystem);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL HOME EVENTS: These events are related to establishing or setting
        /// a home position. It should be recognized that the first mission item in a
        /// mission queue should prepend this position. Just the way ardupilot works.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_VEHICLE_HOME, [this](const int &vehicleID){
            Command_GetHomePosition(vehicleID);
        });

        this->template AddCommandLogic<CommandItem::SpatialHome>(CT::SET_VEHICLE_HOME, [this](const CommandItem::SpatialHome &vehicleHome){
            Command_SetHomePosition(vehicleHome);
        });

    }

public:

    virtual void Request_FullDataSync(const int &targetSystem) = 0;

    virtual void Command_SystemArm(const CommandItem::ActionArm &command) = 0;
    virtual void Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &command) = 0;
    virtual void Command_Land(const CommandItem::SpatialLand &command) = 0;
    virtual void Command_ReturnToLaunch(const CommandItem::SpatialRTL &command) = 0;
    virtual void Command_MissionState(const CommandItem::ActionMissionCommand &command) = 0;
    virtual void Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command) = 0;

    virtual void Command_ChangeSystemMode(const CommandItem::ActionChangeMode &vehicleMode) = 0;

    virtual void Command_UploadMission(const MissionItem::MissionList &missionList) = 0;
    virtual void Command_SetCurrentMission(const Data::MissionKey &key) = 0;
    virtual void Command_GetCurrentMission(const int &targetSystem) = 0;
    virtual void Command_GetMission(const Data::MissionKey &key) = 0;
    virtual void Command_ClearCurrentMission(const int &targetSystem) = 0;

    virtual void Command_GetOnboardAuto(const int &targetSystem) = 0;
    virtual void Command_ClearOnboardAuto(const int &targetSystem) = 0;

    virtual void Command_GetOnboardGuided(const int &targetSystem) = 0;
    virtual void Command_ClearOnboardGuided(const int &targetSystem) = 0;

    virtual void Command_GetHomePosition(const int &vehicleID) = 0;
    virtual void Command_SetHomePosition(const CommandItem::SpatialHome &vehicleHome) = 0;


};

} //end of namespace MaceCore

#endif // ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
