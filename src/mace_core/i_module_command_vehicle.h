#ifndef I_VEHICLE_COMMS_H
#define I_VEHICLE_COMMS_H

#include "data_generic_mission_item/mission_item_components.h"

#include "abstract_module_event_listeners.h"
#include "metadata_vehicle.h"

#include "i_module_topic_events.h"
#include "i_module_events_vehicle.h"

namespace MaceCore
{

enum class VehicleCommands
{
    CHANGE_VEHICLE_ARM,
    CHANGE_VEHICLE_MODE,
    REQUEST_VEHICLE_TAKEOFF,

    SET_CURRENT_MISSION_QUEUE,
    REQUEST_CURRENT_MISSION_QUEUE,
    REQUEST_CLEAR_MISSION_QUEUE,

    SET_CURRENT_GUIDED_QUEUE,
    REQUEST_CURRENT_GUIDED_QUEUE,
    REQUEST_CLEAR_GUIDED_QUEUE,

    REQUEST_VEHICLE_HOME,
    SET_VEHICLE_HOME,

    CREATE_VEHICLE_OBJECT,
    REMOVE_VEHICLE_OBJECT,
    UPDATE_VEHICLE_OBJECT_LIST,
    FOLLOW_NEW_COMMANDS,
    FINISH_AND_FOLLOW_COMMANDS,
    COMMANDS_APPENDED
};


class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandVehicle : public AbstractModule_EventListeners<MetadataVehicle, IModuleEventsVehicle, VehicleCommands>
{
friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandVehicle():
        AbstractModule_EventListeners()
    {
        //These are from MACE Core to modules

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
        /// command and action sequence that accompanies the vheicle. Expect an acknowledgement
        /// or an event to take place when calling these items.
        /////////////////////////////////////////////////////////////////////////

        AddCommandLogic<MissionItem::ActionArm>(VehicleCommands::CHANGE_VEHICLE_ARM, [this](const MissionItem::ActionArm &vehicleArm){
            ChangeVehicleArm(vehicleArm);
        });

        AddCommandLogic<MissionItem::ActionChangeMode>(VehicleCommands::CHANGE_VEHICLE_MODE, [this](const MissionItem::ActionChangeMode &vehicleMode){
            ChangeVehicleOperationalMode(vehicleMode);
        });

        AddCommandLogic<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(VehicleCommands::REQUEST_VEHICLE_TAKEOFF, [this](const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff){
            RequestVehicleTakeoff(vehicleTakeoff);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
        /// This functionality may be pertinent for vehicles not containing a
        /// direct MACE hardware module.
        /////////////////////////////////////////////////////////////////////////

        AddCommandLogic<MissionItem::MissionList>(VehicleCommands::SET_CURRENT_MISSION_QUEUE, [this](const MissionItem::MissionList &missionList){
            SetCurrentMissionQueue(missionList);
        });

        AddCommandLogic<int>(VehicleCommands::REQUEST_CURRENT_MISSION_QUEUE, [this](const int &vehicleID){
            RequestCurrentMissionQueue(vehicleID);
        });

        AddCommandLogic<int>(VehicleCommands::REQUEST_CLEAR_MISSION_QUEUE, [this](const int &vehicleID){
            RequestClearMissionQueue(vehicleID);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
        /// This functionality is pertinent for vehicles that may contain a
        /// MACE HW module, or, vehicles that have timely or ever updating changes.
        /////////////////////////////////////////////////////////////////////////

        AddCommandLogic<MissionItem::MissionList>(VehicleCommands::SET_CURRENT_GUIDED_QUEUE, [this](const MissionItem::MissionList &missionList){
            SetCurrentGuidedQueue(missionList);
        });

        AddCommandLogic<int>(VehicleCommands::REQUEST_CURRENT_GUIDED_QUEUE, [this](const int &vehicleID){
            RequestCurrentGuidedQueue(vehicleID);
        });

        AddCommandLogic<int>(VehicleCommands::REQUEST_CLEAR_GUIDED_QUEUE, [this](const int &vehicleID){
            RequestClearGuidedQueue(vehicleID);
        });


        /////////////////////////////////////////////////////////////////////////
        /// GENERAL HOME EVENTS: These events are related to establishing or setting
        /// a home position. It should be recognized that the first mission item in a
        /// mission queue should prepend this position. Just the way ardupilot works.
        /////////////////////////////////////////////////////////////////////////

        AddCommandLogic<int>(VehicleCommands::REQUEST_VEHICLE_HOME, [this](const int &vehicleID){
            RequestVehicleHomePosition(vehicleID);
        });

        AddCommandLogic<MissionItem::SpatialHome>(VehicleCommands::SET_VEHICLE_HOME, [this](const MissionItem::SpatialHome &vehicleHome){
            SetVehicleHomePosition(vehicleHome);
        });

    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }


public:

    virtual void ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm) = 0;
    virtual void ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode) = 0;
    virtual void RequestVehicleTakeoff(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff) = 0;

    virtual void SetCurrentMissionQueue(const MissionItem::MissionList &missionList) = 0;
    virtual void RequestCurrentMissionQueue(const int &vehicleID) = 0;
    virtual void RequestClearMissionQueue(const int &vehicleID) = 0;

    virtual void SetCurrentGuidedQueue(const MissionItem::MissionList &missionList) = 0;
    virtual void RequestCurrentGuidedQueue(const int &vehicleID) = 0;
    virtual void RequestClearGuidedQueue(const int &vehicleID) = 0;

    virtual void RequestVehicleHomePosition(const int &vehicleID) = 0;
    virtual void SetVehicleHomePosition(const MissionItem::SpatialHome &vehicleHome) = 0;

};


} //END MaceCore Namespace

#endif // I_VEHICLE_COMMS_H
