#ifndef DATA_CONTAINER_MAVLINK_H
#define DATA_CONTAINER_MAVLINK_H


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

public:
    /////////////////////////////////////////////////////////////////////////
    /// MISSION ITEMS: The following hold mission items that either are
    /// reflected when entering the guided mode enabling the vehicle
    /// to be responsive or in autonomous mode.
    /////////////////////////////////////////////////////////////////////////

    MissionItem::MissionList m_CurrentMissionQueue;
    MissionItem::MissionList m_ProposedMissionQueue;

    MissionItem::MissionList m_CurrentGuidedQueue;
    MissionItem::MissionList m_ProposedGuidedQueue;

protected:

    bool heartbeatSeen = false;

    ///////////////////////////////////////////////////////////////////////////////
    /// VEHICLE DATA ITEMS: The following contain information about the direct
    /// state of the vehicle. Each are used in a comparitive operation to
    /// determine if the data has changed and should be published throughout MACE.
    //////////////////////////////////////////////////////////////////////////////

    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> m_CurrentVehicleState;
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Fuel> m_CurrentVehicleFuel;
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> m_CurrentVehicleGPS;
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> m_CurrentVehicleText;


    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> m_CurrentGlobalPosition;
    std::shared_ptr<DataStateTopic::StateLocalPositionTopic> m_CurrentLocalPosition;
    std::shared_ptr<DataStateTopic::StateAttitudeTopic> m_CurrentVehicleAttitude;

};

} //end of namespace DataArdupilot
#endif // DATA_CONTAINER_MAVLINK_H
