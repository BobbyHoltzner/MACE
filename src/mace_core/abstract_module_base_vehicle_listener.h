#ifndef ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
#define ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H

#include "abstract_module_event_listeners.h"
#include "metadata_vehicle.h"

#include "data/mission_key.h"

#define BASE_MODULE_VEHICLE_LISTENER_ENUMS CHANGE_VEHICLE_ARM,CHANGE_VEHICLE_MODE,REQUEST_VEHICLE_TAKEOFF, \
    UPLOAD_MISSION, SET_CURRENT_MISSION, REQUEST_CURRENT_MISSION, REQUEST_MISSION, CLEAR_CURRENT_MISSION,\
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

        this->template AddCommandLogic<MissionItem::ActionArm>(CT::CHANGE_VEHICLE_ARM, [this](const MissionItem::ActionArm &vehicleArm){
            ChangeVehicleArm(vehicleArm);
        });

        this->template AddCommandLogic<MissionItem::ActionChangeMode>(CT::CHANGE_VEHICLE_MODE, [this](const MissionItem::ActionChangeMode &vehicleMode){
            ChangeVehicleOperationalMode(vehicleMode);
        });

        this->template AddCommandLogic<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(CT::REQUEST_VEHICLE_TAKEOFF, [this](const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff){
            Event_RequestVehicleTakeoff(vehicleTakeoff);
        });


        //////////////////////////////////////////////////////////////////////////////////////////////
        /// GENERAL MISSION EVENTS: This functionality may be pertinent for vehicles not containing a
        /// direct MACE hardware module.
        /////////////////////////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<MissionItem::MissionList>(CT::UPLOAD_MISSION, [this](const MissionItem::MissionList &missionList){
            UploadMission(missionList);
        });

        this->template AddCommandLogic<Data::MissionKey>(CT::SET_CURRENT_MISSION, [this](const Data::MissionKey &key){
            SetCurrentMission(key);
        });

        this->template AddCommandLogic<int>(CT::REQUEST_CURRENT_MISSION, [this](const int &targetSystem){
            GetCurrentMission(targetSystem);
        });

        this->template AddCommandLogic<Data::MissionKey>(CT::REQUEST_MISSION, [this](const Data::MissionKey &key){
            GetMission(key);
        });

        this->template AddCommandLogic<int>(CT::CLEAR_CURRENT_MISSION, [this](const int &targetSystem){
            ClearCurrentMission(targetSystem);
        });

        ////////////////////////////////////////////////////////////////////////////
        /// GENERAL AUTO EVENTS: This is implying for guided mode of the vehicle.
        /// This functionality is pertinent for vehicles that may contain a
        /// MACE HW module, or, vehicles that have timely or ever updating changes.
        ////////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_ONBOARD_AUTO_MISSION, [this](const int &targetSystem){
            GetOnboardAuto(targetSystem);
        });

        this->template AddCommandLogic<int>(CT::CLEAR_ONBOARD_AUTO_MISSION, [this](const int &targetSystem){
            ClearOnboardAuto(targetSystem);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
        /// This functionality is pertinent for vehicles that may contain a
        /// MACE HW module, or, vehicles that have timely or ever updating changes.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_ONBOARD_GUIDED_MISSION, [this](const int &targetSystem){
            GetOnboardGuided(targetSystem);
        });

        this->template AddCommandLogic<int>(CT::CLEAR_ONBOARD_GUIDED_MISSION, [this](const int &targetSystem){
            ClearOnboardGuided(targetSystem);
        });

        /////////////////////////////////////////////////////////////////////////
        /// GENERAL HOME EVENTS: These events are related to establishing or setting
        /// a home position. It should be recognized that the first mission item in a
        /// mission queue should prepend this position. Just the way ardupilot works.
        /////////////////////////////////////////////////////////////////////////

        this->template AddCommandLogic<int>(CT::REQUEST_VEHICLE_HOME, [this](const int &vehicleID){
            Event_GetHomePosition(vehicleID);
        });

        this->template AddCommandLogic<MissionItem::SpatialHome>(CT::SET_VEHICLE_HOME, [this](const MissionItem::SpatialHome &vehicleHome){
            Event_SetHomePosition(vehicleHome);
        });

    }

public:

    virtual void ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm) = 0;
    virtual void ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode) = 0;
    virtual void Event_RequestVehicleTakeoff(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff) = 0;

    virtual void UploadMission(const MissionItem::MissionList &missionList) = 0;
    virtual void SetCurrentMission(const Data::MissionKey &key) = 0;
    virtual void GetCurrentMission(const int &targetSystem) = 0;
    virtual void GetMission(const Data::MissionKey &key) = 0;
    virtual void ClearCurrentMission(const int &targetSystem) = 0;

    virtual void GetOnboardAuto(const int &targetSystem) = 0;
    virtual void ClearOnboardAuto(const int &targetSystem) = 0;

    virtual void GetOnboardGuided(const int &targetSystem) = 0;
    virtual void ClearOnboardGuided(const int &targetSystem) = 0;

    virtual void Event_GetHomePosition(const int &vehicleID) = 0;
    virtual void Event_SetHomePosition(const MissionItem::SpatialHome &vehicleHome) = 0;


};

} //end of namespace MaceCore

#endif // ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
