#include "mace_data.h"

namespace MaceCore{

void MaceData::updateCurrentMissionList(const MissionItem::MissionList missionList)
{
//    std::lock_guard<std::mutex> guard(m_VehicleCurrentMissionMUTEX);
//    MissionItem::MissionList::MissionType relevantQueue = missionList.getMissionType();
//    int systemID = missionList.getVehicleID();

//    switch(relevantQueue){
//    case MissionItem::MissionList::AUTO_ACTUAL:
//    {
//        try{
//            m_VehicleCurrentMission[systemID][MissionItem::MissionList::AUTO_ACTUAL] = missionList;
//        }catch(const std::out_of_range &oor){
//            std::cout<<"MACEDATA has requested an OOR on current auto mission map"<<std::endl;
//        }
//        break;
//    }
//    case MissionItem::MissionList::GUIDED_ACTUAL:
//    default:
//    {
//        try{
//            m_VehicleCurrentMission[systemID][MissionItem::MissionList::GUIDED_ACTUAL] = missionList;
//        }catch(const std::out_of_range &oor){
//            std::cout<<"MACEDATA has requested an OOR on current guided mission map"<<std::endl;
//        }
//        break;
//    }
//    }
}

void MaceData::updateProposedMissionList(const MissionItem::MissionList missionList)
{
//    std::lock_guard<std::mutex> guard(m_VehicleProposedMissionMUTEX);
//    MissionItem::MissionList::MissionType relevantQueue = missionList.getMissionType();
//    int systemID = missionList.getVehicleID();

//    switch(relevantQueue){
//    case MissionItem::MissionList::AUTO_PROPOSED:
//    {
//        try{
//            m_VehicleProposedMission[systemID][MissionItem::MissionList::AUTO_PROPOSED] = missionList;
//        }catch(const std::out_of_range &oor){
//            std::cout<<"MACEDATA has requested an OOR on current auto mission map"<<std::endl;
//        }
//        break;
//    }
//    case MissionItem::MissionList::GUIDED_PROPOSED:
//    default:
//    {
//        try{
//            m_VehicleProposedMission[systemID][MissionItem::MissionList::GUIDED_PROPOSED] = missionList;
//        }catch(const std::out_of_range &oor){
//            std::cout<<"MACEDATA has requested an OOR on current guided mission map"<<std::endl;
//        }
//        break;
//    }
//    }
}

MissionItem::MissionList MaceData::getMissionList(const int &systemID, const MissionItem::MissionList::MissionType missionType) const
{
    MissionItem::MissionList newList;
    switch(missionType){
    case MissionItem::MissionList::AUTO_ACTUAL:
    case MissionItem::MissionList::GUIDED_ACTUAL:
    {
        std::lock_guard<std::mutex> guardCurrentMission(m_VehicleCurrentMissionMUTEX);
        try{
            newList = m_VehicleCurrentMission.at(systemID).at(missionType);
        }catch(const std::out_of_range &oor){
            std::cout<<"MACEDATA has requested an OOR on proposed mission map"<<std::endl;
        }
        break;
    }
    case MissionItem::MissionList::AUTO_PROPOSED:
    case MissionItem::MissionList::GUIDED_PROPOSED:
    default:
    {
        std::lock_guard<std::mutex> guardProposedMission(m_VehicleProposedMissionMUTEX);
        try{
            newList = m_VehicleProposedMission.at(systemID).at(missionType);
        }catch(const std::out_of_range &oor){
            std::cout<<"MACEDATA has requested an OOR on proposed mission map"<<std::endl;
        }
        break;
    }
    }
    return newList;
}

}
