#ifndef ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
#define ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H

#include "abstract_module_event_listeners.h"

#include "metadata_vehicle.h"

#define BASE_MODULE_VEHICLE_LISTENER_ENUMS CHANGE_VEHICLE_ARM,CHANGE_VEHICLE_MODE,REQUEST_VEHICLE_TAKEOFF,SET_CURRENT_MISSION_QUEUE,REQUEST_CURRENT_MISSION_QUEUE,REQUEST_CLEAR_MISSION_QUEUE,SET_CURRENT_GUIDED_QUEUE,REQUEST_CURRENT_GUIDED_QUEUE,REQUEST_CLEAR_GUIDED_QUEUE,REQUEST_VEHICLE_HOME,SET_VEHICLE_HOME,FOLLOW_NEW_COMMANDS,FINISH_AND_FOLLOW_COMMANDS,COMMANDS_APPENDED

//#define BASE_MODULE_VEHICLE_LISTENER_ENUMS NEW_VEHICLE, REMOVE_VEHICLE, UPDATED_POSITION_DYNAMICS, UPDATED_ATTITUDE_DYNAMICS, UPDATED_VEHICLE_LIFE

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

        this->template AddCommandLogic<MissionItem::ActionArm>(CT::CHANGE_VEHICLE_ARM, [this](const MissionItem::ActionArm &vehicleArm){
            ChangeVehicleArm(vehicleArm);
        });

        this->template AddCommandLogic<MissionItem::ActionChangeMode>(CT::CHANGE_VEHICLE_MODE, [this](const MissionItem::ActionChangeMode &vehicleMode){
            ChangeVehicleOperationalMode(vehicleMode);
        });

        this->template AddCommandLogic<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(CT::REQUEST_VEHICLE_TAKEOFF, [this](const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff){
            RequestVehicleTakeoff(vehicleTakeoff);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
        /// This functionality may be pertinent for vehicles not containing a
        /// direct MACE hardware module.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<MissionItem::MissionList>(CT::SET_CURRENT_MISSION_QUEUE, [this](const MissionItem::MissionList &missionList){
            SetCurrentMissionQueue(missionList);
        });

        this->template AddCommandLogic<int>(CT::REQUEST_CURRENT_MISSION_QUEUE, [this](const int &vehicleID){
            RequestCurrentMissionQueue(vehicleID);
        });

        this->template AddCommandLogic<int>(CT::REQUEST_CLEAR_MISSION_QUEUE, [this](const int &vehicleID){
            RequestClearMissionQueue(vehicleID);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
        /// This functionality is pertinent for vehicles that may contain a
        /// MACE HW module, or, vehicles that have timely or ever updating changes.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<MissionItem::MissionList>(CT::SET_CURRENT_GUIDED_QUEUE, [this](const MissionItem::MissionList &missionList){
            SetCurrentGuidedQueue(missionList);
        });

        this->template AddCommandLogic<int>(CT::REQUEST_CURRENT_GUIDED_QUEUE, [this](const int &vehicleID){
            RequestCurrentGuidedQueue(vehicleID);
        });

        this->template AddCommandLogic<int>(CT::REQUEST_CLEAR_GUIDED_QUEUE, [this](const int &vehicleID){
            RequestClearGuidedQueue(vehicleID);
        });


        /////////////////////////////////////////////////////////////////////////
        /// GENERAL HOME EVENTS: These events are related to establishing or setting
        /// a home position. It should be recognized that the first mission item in a
        /// mission queue should prepend this position. Just the way ardupilot works.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_VEHICLE_HOME, [this](const int &vehicleID){
            RequestVehicleHomePosition(vehicleID);
        });

        this->template AddCommandLogic<MissionItem::SpatialHome>(CT::SET_VEHICLE_HOME, [this](const MissionItem::SpatialHome &vehicleHome){
            SetVehicleHomePosition(vehicleHome);
        });

    }

public:

    virtual void ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm) = 0;
    virtual void ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode) = 0;
    virtual void RequestVehicleTakeoff(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff) = 0;
//    virtual void RequestVehicleLand(const MissionItem::SpatialLand<DataState::StateGlobalPosition> &vehicleLand) = 0;
//    virtual void RequestVehicleRTL(const MissionItem::SpatialRTL &vehicleRTL) = 0;

    virtual void RequestVehicleHomePosition(const int &vehicleID) = 0;
    virtual void SetVehicleHomePosition(const MissionItem::SpatialHome &vehicleHome) = 0;

    virtual void SetCurrentMissionQueue(const MissionItem::MissionList &missionList) = 0;
    virtual void RequestCurrentMissionQueue(const int &vehicleID) = 0;
    virtual void RequestClearMissionQueue(const int &vehicleID) = 0;

    virtual void SetCurrentGuidedQueue(const MissionItem::MissionList &missionList) = 0;
    virtual void RequestCurrentGuidedQueue(const int &vehicleID) = 0;
    virtual void RequestClearGuidedQueue(const int &vehicleID) = 0;




};

} //end of namespace MaceCore

#endif // ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
