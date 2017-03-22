#include "mace_data.h"

namespace MaceCore{

void MaceData::updateMissionList(const MissionItem::MissionList missionList, const Data::MissionMap &relevantQueue)
{
    std::lock_guard<std::mutex> guard(m_VehicleMissionMutex);

    int systemID = missionList.getVehicleID();

    switch(relevantQueue){
    case Data::MissionMap::PROPOSED:
    {
        try{
            m_VehicleProposedMissionMap[systemID] = missionList;
        }catch(const std::out_of_range &oor){
            std::cout<<"MACEDATA has requested an OOR on proposed mission map"<<std::endl;
        }
        break;
    }
    case Data::MissionMap::CURRENT:
    default:
    {
        try{
            m_VehicleCurrentMissionMap[systemID] = missionList;
        }catch(const std::out_of_range &oor){
            std::cout<<"MACEDATA has requested an OOR on proposed mission map"<<std::endl;
        }
        break;
    }
    }
}

MissionItem::MissionList MaceData::getMissionList(const int &systemID, const Data::MissionMap &relevantQueue)
{
    std::lock_guard<std::mutex> guard(m_VehicleMissionMutex);

    MissionItem::MissionList newList;
    switch(relevantQueue){
    case Data::MissionMap::PROPOSED:
    {
        try{
            newList = m_VehicleProposedMissionMap.at(systemID);
        }catch(const std::out_of_range &oor){
            std::cout<<"MACEDATA has requested an OOR on proposed mission map"<<std::endl;
        }
        break;
    }
    case Data::MissionMap::CURRENT:
    default:
    {
        try{
            newList = m_VehicleCurrentMissionMap.at(systemID);
        }catch(const std::out_of_range &oor){
            std::cout<<"MACEDATA has requested an OOR on proposed mission map"<<std::endl;
        }
        break;
    }
    }
    return newList;
}

}
