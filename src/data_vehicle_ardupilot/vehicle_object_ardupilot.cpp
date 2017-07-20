#include "vehicle_object_ardupilot.h"

namespace DataARDUPILOT {

VehicleObject_ARDUPILOT::VehicleObject_ARDUPILOT(const int &vehicleID, const int &systemID, const int &systemComp):
    m_VehicleID(vehicleID), m_SystemID(systemID), m_SystemComp(systemComp),
    Container_ARDUPILOTTOMACE(vehicleID),Container_MACETOARDUPILOT(systemID,systemComp)
{
    data = new DataContainer_ARDUPILOT();
    parser = new ARDUPILOTParser(data);
}

std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> VehicleObject_ARDUPILOT::GetAllTopicData()
{
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> ptrHeartbeat = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>(data->vehicleHeartbeat.get());
    rtnVector.push_back(ptrHeartbeat);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrMode = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>(data->vehicleMode.get());
    rtnVector.push_back(ptrMode);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> ptrArm = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>(data->vehicleArm.get());
    rtnVector.push_back(ptrArm);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> ptrFuel = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>(data->vehicleFuel.get());
    rtnVector.push_back(ptrFuel);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPSStatus = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(data->vehicleGPSStatus.get());
    rtnVector.push_back(ptrGPSStatus);

    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> ptrGlobalPosition = std::make_shared<DataStateTopic::StateGlobalPositionTopic>(data->vehicleGlobalPosition.get());
    rtnVector.push_back(ptrGlobalPosition);
    std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> ptrGlobalPositionEx = std::make_shared<DataStateTopic::StateGlobalPositionExTopic>(data->vehicleGlobalPositionEx.get());
    rtnVector.push_back(ptrGlobalPositionEx);
    std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>(data->vehicleLocalPosition.get());
    rtnVector.push_back(ptrLocalPosition);
    std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(data->vehicleAttitude.get());
    rtnVector.push_back(ptrAttitude);
    std::shared_ptr<DataStateTopic::StateAirspeedTopic> ptrAirspeed = std::make_shared<DataStateTopic::StateAirspeedTopic>(data->vehicleAirspeed.get());
    rtnVector.push_back(ptrAirspeed);

    return rtnVector;
}


bool VehicleObject_ARDUPILOT::generateBasicGuidedMessage(const std::shared_ptr<CommandItem::AbstractCommandItem> &missionItem, const uint8_t &chan, mavlink_message_t &msg)
{
//    switch(missionItem->getCommandType())
//    {
//    case(Data::CommandItemType::CI_NAV_WAYPOINT):
//    {
//        if(missionItem->getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
//        {
//            std::shared_ptr<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
//            CommandItem::SpatialWaypoint<DataState::StateGlobalPosition> baseItem = *castItem.get();
//            mavlink_mission_item_t newItem = Waypoint_MACETOMAVLINK(baseItem,0,0);
//            newItem.current = 2;
//            msg = packMissionItem(newItem,chan);
//            return true;
//        }else{
//            return false;
//        }
//        break;
//    }
//    default:
//        return false;
//    } //end of switch statement
}

}//end of namespace DataARDUPILOT
