#include "mace_data.h"

namespace MaceCore{

int MaceData::getAvailableMissionID(const int &systemID)
{
    int prevID = 0;
    std::lock_guard<std::mutex> guard(MUTEXMissionID);
    std::map<int,int>::iterator it;
    it = mapMissionID.find(systemID);

    if(it != mapMissionID.end()) //this implies that there is something that already exists
    {
        prevID = it->second;
        prevID++;
    }
    return prevID;
}

void MaceData::updateMissionID(const int &systemID, const int &prevID, const int &newID)
{
    std::lock_guard<std::mutex> guard(MUTEXAvailableMissions);
    int delta = newID - prevID;
    try{
        std::map<Data::MissionKey,MissionItem::MissionList> correctedMissionMap;

        std::map<Data::MissionKey,MissionItem::MissionList> availableMissions = mapAvailableMissions[systemID];

        for (std::map<Data::MissionKey,MissionItem::MissionList>::iterator it=availableMissions.begin(); it!=availableMissions.end(); ++it){

            Data::MissionKey missionKey = it->first;
            MissionItem::MissionList mission = it->second;

            int missionID = missionKey.m_missionID;

            if(missionID>=prevID)
            {
                mission.setMissionID(missionID + delta);
            }
            correctedMissionMap[mission.getMissionKey()] = mission;
        }
        mapAvailableMissions[systemID] = correctedMissionMap;
    }catch(const std::out_of_range &oor){
        std::cout<<"MACEDATA has requested an OOR in updateMissionID of mace_data.cpp"<<std::endl;
    }
}


MissionItem::MissionList MaceData::appenedAvailableMissionMap(const int &newSystemID, const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList correctedMissionList = missionList;
    correctedMissionList.setVehicleID(newSystemID);
    int updatedMissionID = getAvailableMissionID(newSystemID);
    correctedMissionList.setMissionID(updatedMissionID);

    std::lock_guard<std::mutex> guard(MUTEXAvailableMissions);
    mapAvailableMissions[newSystemID][correctedMissionList.getMissionKey()] = correctedMissionList;
    return correctedMissionList;
}

MissionItem::MissionList MaceData::appenedAvailableMissionMap(const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList correctedMissionList = missionList;
    int systemID = missionList.getVehicleID();
    int updatedMissionID = getAvailableMissionID(systemID);
    correctedMissionList.setMissionID(updatedMissionID);

    std::lock_guard<std::mutex> guard(MUTEXAvailableMissions);
    mapAvailableMissions[correctedMissionList.getVehicleID()][correctedMissionList.getMissionKey()] = correctedMissionList;
    return correctedMissionList;
}


bool MaceData::updateCurrentVehicleMission(const Data::MissionKey &missionKey)
{
    int systemID = missionKey.m_targetID;
    std::lock_guard<std::mutex> guardAvailable(MUTEXAvailableMissions);
    std::map<int,std::map<Data::MissionKey, MissionItem::MissionList>>::iterator it;
    it = mapAvailableMissions.find(systemID);

    if(it != mapAvailableMissions.end()) //means the system ID was in there
    {
        std::map<Data::MissionKey, MissionItem::MissionList> vehicleList = it->second;
        std::map<Data::MissionKey, MissionItem::MissionList>::iterator itSub;
        itSub = vehicleList.find(missionKey);
        if(itSub != vehicleList.end()) //means the missionKey was in there
        {
            std::lock_guard<std::mutex> guardCurrent(MUTEXCurrentMissions);
            mapCurrentMissions[missionKey] = itSub->second;
            return true;
        }
    }
    return false;
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
