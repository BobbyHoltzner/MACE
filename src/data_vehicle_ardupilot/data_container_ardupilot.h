#ifndef DATA_CONTAINER_ARDUPILOT_H
#define DATA_CONTAINER_ARDUPILOT_H

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"
#include "data_generic_item_topic/data_generic_item_topic_fuel.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "components/vehicle_flightMode.h"

namespace DataArdupilot
{

class DataContainer
{
public:
    DataContainer();

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

public:

    bool heartbeatSeen = false;

    ///////////////////////////////////////////////////////////////////////////////
    /// VEHICLE DATA ITEMS: The following contain information about the direct
    /// state of the vehicle. Each are used in a comparitive operation to
    /// determine if the data has changed and should be published throughout MACE.
    //////////////////////////////////////////////////////////////////////////////

    std::shared_ptr<VehicleFlightMode> m_CurrentArduVehicleState;
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Fuel> m_CurrentArduVehicleFuel;
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> m_CurrentArduVehicleGPS;
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> m_CurrentArduVehicleText;


    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> m_CurrentArduGlobalPosition;
    std::shared_ptr<DataStateTopic::StateLocalPositionTopic> m_CurrentArduLocalPosition;
    std::shared_ptr<DataStateTopic::StateAttitudeTopic> m_CurrentArduVehicleAttitude;

};

} //end of namespace DataArdupilot

#endif // DATA_CONTAINER_ARDUPILOT_H
