#ifndef DATA_CONTAINER_MAVLINK_H
#define DATA_CONTAINER_MAVLINK_H

#include <mutex>

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
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
protected:
    mutable std::mutex missionMutex;
    MissionItem::MissionList m_CurrentMissionQueue;
    MissionItem::MissionList m_ProposedMissionQueue;

    MissionItem::MissionList m_CurrentGuidedQueue;
    MissionItem::MissionList m_ProposedGuidedQueue;

    public:

    Data::MissionKey proposedMissionConfirmed(){
        m_CurrentMissionQueue = m_ProposedMissionQueue;
        m_ProposedMissionQueue.clearQueue();
        return m_CurrentMissionQueue.getMissionKey();
    }

    void setCurrentMission(const MissionItem::MissionList &missionList)
    {
        std::lock_guard<std::mutex> guard(missionMutex);
        switch(missionList.getMissionType())
        {
        case Data::MissionType::AUTO:
        {
            m_CurrentMissionQueue = missionList;
            break;
        }
        case Data::MissionType::GUIDED:
        {
            m_CurrentGuidedQueue = missionList;
            break;
        }
        default:
            break;
        }
    }

    void setProposedMission(const MissionItem::MissionList &missionList)
    {
        std::lock_guard<std::mutex> guard(missionMutex);
        switch(missionList.getMissionType())
        {
        case Data::MissionType::AUTO:
        {
            m_ProposedMissionQueue = missionList;
            break;
        }
        case Data::MissionType::GUIDED:
        {
            m_ProposedGuidedQueue = missionList;
            break;
        }
        default:
            break;
        }
    }

    MissionItem::MissionList getCurrentMission(const Data::MissionType &type){
        std::lock_guard<std::mutex> guard(missionMutex);
        MissionItem::MissionList rtnList;
        switch(type){
        case Data::MissionType::AUTO:
        {
            rtnList = m_CurrentMissionQueue;
            break;
        }
        case Data::MissionType::GUIDED:
        {
            rtnList = m_CurrentGuidedQueue;
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
        std::lock_guard<std::mutex> guard(missionMutex);
        MissionItem::MissionList rtnList;
        switch(type){
        case Data::MissionType::AUTO:
        {
            rtnList = m_ProposedMissionQueue;
            break;
        }
        case Data::MissionType::GUIDED:
        {
            rtnList = m_ProposedGuidedQueue;
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
        DataGenericItem::DataGenericItem_FlightMode m_CurrentVehicleState;
        DataGenericItem::DataGenericItem_Fuel m_CurrentVehicleFuel;
        DataGenericItem::DataGenericItem_GPS m_CurrentVehicleGPS;
        DataGenericItem::DataGenericItem_Text m_CurrentVehicleText;

    //    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> m_CurrentVehicleState;
    //    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Fuel> m_CurrentVehicleFuel;
    //    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> m_CurrentVehicleGPS;
    //    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> m_CurrentVehicleText;

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

    void setFlightMode(const DataGenericItem::DataGenericItem_FlightMode &info)
    {
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        m_CurrentVehicleState = info;
    }

    DataGenericItem::DataGenericItem_FlightMode getFlightMode() const{
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        return m_CurrentVehicleState;
    }

    void setFuel(const DataGenericItem::DataGenericItem_Fuel &info)
    {
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        m_CurrentVehicleFuel = info;
    }

    DataGenericItem::DataGenericItem_Fuel getFuel() const{
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        return m_CurrentVehicleFuel;
    }

    void setGPS(const DataGenericItem::DataGenericItem_GPS &info)
    {
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        m_CurrentVehicleGPS = info;
    }

    DataGenericItem::DataGenericItem_GPS getGPS() const{
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        return m_CurrentVehicleGPS;
    }

    void setText(const DataGenericItem::DataGenericItem_Text &info)
    {
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        m_CurrentVehicleText = info;
    }

    DataGenericItem::DataGenericItem_Text getText() const{
        std::lock_guard<std::mutex> guard(genericTopicMutex);
        return m_CurrentVehicleText;
    }


    ///////////////////////////////////////////////////////////////////////////////
    /// DATA STATE ITEMS
    //////////////////////////////////////////////////////////////////////////////
protected:
    mutable std::mutex stateTopicMutex;
    DataState::StateGlobalPosition m_CurrentGlobalPosition;
    DataState::StateGlobalPositionEx m_CurrentGlobalPositionEx;
    DataState::StateLocalPosition m_CurrentLocalPosition;
    DataState::StateAttitude m_CurrentVehicleAttitude;

//    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> m_CurrentGlobalPosition;
//    std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> m_CurrentGlobalPositionEx;
//    std::shared_ptr<DataStateTopic::StateLocalPositionTopic> m_CurrentLocalPosition;
//    std::shared_ptr<DataStateTopic::StateAttitudeTopic> m_CurrentVehicleAttitude;

public:
    void setGlobalPos(const DataState::StateGlobalPosition &info)
    {
        std::lock_guard<std::mutex> guard(stateTopicMutex);
        m_CurrentGlobalPosition = info;
    }

    DataState::StateGlobalPosition getGlobalPos() const{
        std::lock_guard<std::mutex> guard(stateTopicMutex);
        return  m_CurrentGlobalPosition;
    }

    void setGlobalPosEx(const DataState::StateGlobalPositionEx &info)
    {
        std::lock_guard<std::mutex> guard(stateTopicMutex);
        m_CurrentGlobalPositionEx = info;
    }

    DataState::StateGlobalPositionEx getGlobalPosEx() const{
        std::lock_guard<std::mutex> guard(stateTopicMutex);
        return m_CurrentGlobalPositionEx;
    }

    void setLocalPosition(const DataState::StateLocalPosition &info)
    {
        std::lock_guard<std::mutex> guard(stateTopicMutex);
        m_CurrentLocalPosition = info;
    }

    DataState::StateLocalPosition getLocalPosition() const{
        std::lock_guard<std::mutex> guard(stateTopicMutex);
        return m_CurrentLocalPosition;
    }

    void setAttitude(const DataState::StateAttitude &info)
    {
        std::lock_guard<std::mutex> guard(stateTopicMutex);
        m_CurrentVehicleAttitude = info;
    }

    DataState::StateAttitude getAttitude() const{
        std::lock_guard<std::mutex> guard(stateTopicMutex);
        return m_CurrentVehicleAttitude;
    }

    ///////////////////////////////////////////////////////////////////////////////
    /// DATA MISSION TOPIC ITEMS
    //////////////////////////////////////////////////////////////////////////////
public:
    mutable std::mutex missionTopicMutex;
    std::shared_ptr<MissionTopic::MissionHomeTopic> m_MissionHome;
    std::shared_ptr<MissionTopic::MissionItemReachedTopic> m_MissionItemReached;
    std::shared_ptr<MissionTopic::MissionItemCurrentTopic> m_MissionItemCurrent;
};

} //end of namespace DataArdupilot
#endif // DATA_CONTAINER_MAVLINK_H
