#ifndef DATA_CONTAINER_MAVLINK_H
#define DATA_CONTAINER_MAVLINK_H

#include <mutex>

#include "data/data_get_set_notifier.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK
{

class DataContainer_MAVLINK
{
public:
    DataContainer_MAVLINK();

    /////////////////////////////////////////////////////////////////////////
    /// MISSION ITEMS: The following hold mission items that either are
    /// reflected when entering the guided mode enabling the vehicle
    /// to be responsive or in autonomous mode.
    /////////////////////////////////////////////////////////////////////////
public:
    Data::DataGetSetNotifier<MissionItem::MissionList> currentAutoMission;
    Data::DataGetSetNotifier<MissionItem::MissionList> proposedAutoMission;
    Data::DataGetSetNotifier<MissionItem::MissionList> currentGuidedMission;
    Data::DataGetSetNotifier<MissionItem::MissionList> proposedGuidedMission;

    public:

    Data::MissionKey proposedMissionConfirmed(){
        MissionItem::MissionList propList = proposedAutoMission.get();
        currentAutoMission.set(propList);
        propList.clearQueue();
        proposedAutoMission.set(propList);
        return propList.getMissionKey();
    }

    void setCurrentMission(const MissionItem::MissionList &missionList)
    {
        switch(missionList.getMissionType())
        {
        case Data::MissionType::AUTO:
        {
            currentAutoMission.set(missionList);
            break;
        }
        case Data::MissionType::GUIDED:
        {
            currentGuidedMission.set(missionList);
            break;
        }
        default:
            break;
        }
    }

    void setProposedMission(const MissionItem::MissionList &missionList)
    {
        switch(missionList.getMissionType())
        {
        case Data::MissionType::AUTO:
        {
            proposedAutoMission.set(missionList);
            break;
        }
        case Data::MissionType::GUIDED:
        {
            proposedGuidedMission.set(missionList);
            break;
        }
        default:
            break;
        }
    }

    MissionItem::MissionList Command_GetCurrentMission(const Data::MissionType &type){
        MissionItem::MissionList rtnList;
        switch(type){
        case Data::MissionType::AUTO:
        {
            rtnList = currentAutoMission.get();
            break;
        }
        case Data::MissionType::GUIDED:
        {
            rtnList = currentGuidedMission.get();
            break;
        }
        default:
        {
            break;
        }
        }
        return rtnList;
    }

    MissionItem::MissionList getProposedMission(const Data::MissionType &type){
        MissionItem::MissionList rtnList;
        switch(type){
        case Data::MissionType::AUTO:
        {
            rtnList = proposedAutoMission.get();
            break;
        }
        case Data::MissionType::GUIDED:
        {
            rtnList = proposedGuidedMission.get();
            break;
        }
        default:
        {
            break;
        }
        }
        return rtnList;
    }

    ///////////////////////////////////////////////////////////////////////////////
    /// VEHICLE DATA ITEMS: The following contain information about the direct
    /// state of the vehicle. Each are used in a comparitive operation to
    /// determine if the data has changed and should be published throughout MACE.
    //////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////////
    /// DATA GENERIC ITEMS
    //////////////////////////////////////////////////////////////////////////////
protected:
    mutable std::mutex genericTopicMutex;
    bool heartbeatSeen = false;

public:
        Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_Heartbeat> vehicleHeartbeat;
        Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_FlightMode> vehicleMode;
        Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_SystemArm> vehicleArm;
        Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_FlightMode> vehicleState;
        Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_Battery> vehicleFuel;
        Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_GPS> vehicleGPSStatus;
        Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_Text> vehicleTextAlert;

public:

    void setHeartbeatSeen(const bool &info)
    {
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        heartbeatSeen = info;
    }

    bool getHearbeatSeen() const{
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        return heartbeatSeen;
    }

    ///////////////////////////////////////////////////////////////////////////////
    /// DATA STATE ITEMS
    //////////////////////////////////////////////////////////////////////////////
public:
    Data::DataGetSetNotifier<DataState::StateGlobalPosition> vehicleGlobalPosition;
    Data::DataGetSetNotifier<DataState::StateGlobalPositionEx> vehicleGlobalPositionEx;
    Data::DataGetSetNotifier<DataState::StateLocalPosition> vehicleLocalPosition;
    Data::DataGetSetNotifier<DataState::StateAttitude> vehicleAttitude;
    Data::DataGetSetNotifier<DataState::StateAirspeed> vehicleAirspeed;


    ///////////////////////////////////////////////////////////////////////////////
    /// DATA MISSION TOPIC ITEMS
    //////////////////////////////////////////////////////////////////////////////
public:
    Data::DataGetSetNotifier<CommandItem::SpatialHome> vehicleHome;
    std::shared_ptr<MissionTopic::MissionItemReachedTopic> m_MissionItemReached;
    std::shared_ptr<MissionTopic::MissionItemCurrentTopic> m_MissionItemCurrent;
};

} //end of namespace DataArdupilot
#endif // DATA_CONTAINER_MAVLINK_H
