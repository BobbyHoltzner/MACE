#include "mace_data.h"

namespace MaceCore{


void MaceData::updateCOMPLETEMissionList(const MissionItem::MissionList missionList)
{
    std::lock_guard<std::mutex> guardComplete(COMPLETEMissionMUTEX);
    int systemID = missionList.getVehicleID();
    Data::MissionType missionType = missionList.getMissionType();
    m_COMPLETEMission[systemID][missionType] = missionList;

    //Now that the list is complete, remove the outdated/associated
    //one from the incomplete queue
    std::lock_guard<std::mutex> guardIncomplete(INCOMPLETEMissionMUTEX);
    std::map<int, std::map<Data::MissionType,MissionItem::MissionList>>::iterator it;
    it = m_INCOMPLETEMission.find(systemID);
    if(it != m_INCOMPLETEMission.end())
    {
        std::map<Data::MissionType,MissionItem::MissionList>::iterator innerIt;
        innerIt = m_INCOMPLETEMission.at(systemID).find(missionType);
        if(innerIt != m_INCOMPLETEMission.at(systemID).end())
        {
            m_INCOMPLETEMission.at(systemID).erase(innerIt);
            if(m_INCOMPLETEMission.at(systemID).size() == 0)
            {
                m_INCOMPLETEMission.erase(systemID);
            }
        }
    }
}

void MaceData::updateINCOMPLETEMissionList(const MissionItem::MissionList missionList)
{
    std::lock_guard<std::mutex> guard(INCOMPLETEMissionMUTEX);
    int systemID = missionList.getVehicleID();
    Data::MissionType missionType = missionList.getMissionType();
    m_INCOMPLETEMission[systemID][missionType] = missionList;
}

bool MaceData::getMissionList(MissionItem::MissionList &newList, const int &systemID, const MissionItem::MissionList::MissionListState &missionState, const Data::MissionType &missionType) const
{
    switch(missionState)
    {
    case MissionItem::MissionList::COMPLETE:
    {
        std::lock_guard<std::mutex> guard(COMPLETEMissionMUTEX);
        try{
            newList = m_COMPLETEMission.at(systemID).at(missionType);
        }catch(const std::out_of_range &oor){
            std::cout<<"MACEDATA has requested an OOR on complete mission map"<<std::endl;
            return false;
        }
        break;
    }
    case MissionItem::MissionList::INCOMPLETE:
    {
        std::lock_guard<std::mutex> guard(INCOMPLETEMissionMUTEX);
        try{
            newList = m_INCOMPLETEMission.at(systemID).at(missionType);
        }catch(const std::out_of_range &oor){
            std::cout<<"MACEDATA has requested an OOR on incomplete mission map"<<std::endl;
            return false;
        }
        break;
    }
    }

    return true;
}

}
