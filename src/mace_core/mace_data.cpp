#include "mace_data.h"

namespace MaceCore{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MISSION METHODS | PUSHING TO MACE DATA
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

MissionItem::MissionKey MaceData::appendAssociatedMissionMap(const int &newSystemID, const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList correctedMissionList = missionList;
    correctedMissionList.setVehicleID(newSystemID);
    int updatedMissionID = getAvailableMissionID(missionList.getMissionKey());
    correctedMissionList.setMissionID(updatedMissionID);

    std::lock_guard<std::mutex> guard(MUTEXMissions);
    mapMissions[correctedMissionList.getMissionKey()] = correctedMissionList;
    return correctedMissionList.getMissionKey();
}

MissionItem::MissionKey MaceData::appendAssociatedMissionMap(const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList correctedMissionList = missionList;
    int systemID = missionList.getVehicleID();
    int updatedMissionID = getAvailableMissionID(missionList.getMissionKey());
    correctedMissionList.setMissionID(updatedMissionID);

    std::lock_guard<std::mutex> guard(MUTEXMissions);
    mapMissions[correctedMissionList.getMissionKey()] = correctedMissionList;
    return correctedMissionList.getMissionKey();
}

int MaceData::getAvailableMissionID(const MissionItem::MissionKey &key)
{
    int prevID = 0;
    std::lock_guard<std::mutex> guard(MUTEXMissionID);
    std::map<int,std::map<int,int>>::iterator it;
    it = mapMissionID.find(key.m_systemID);

    if(it != mapMissionID.end()) //this implies that there is a vehicle in the queue
    {
        std::map<int,int> internalMap = mapMissionID.at(key.m_systemID);
        std::map<int,int>::iterator itInternal;
        itInternal = internalMap.find(key.m_creatorID);

        if(itInternal != internalMap.end()) //this implies that there is a creator in the queue
        {
            prevID = itInternal->second;
            if(prevID>=key.m_missionID)
                prevID++;
            else
                prevID = key.m_missionID;
        }
        //if something already exists we need to compare the value

    }
    mapMissionID[key.m_systemID][key.m_creatorID] = prevID;
    return prevID;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MISSION METHODS | HANDLING NEW RECEIVING QUEUES
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
The following methods aid getting the mission list from the mace data class. The following methods aid getting
the current mission object and keys.
*/
bool MaceData::updateCurrentMissionItem(const MissionItem::MissionItemCurrent &current)
{
    MissionItem::MissionKey key = current.getMissionKey();
    int index = current.getMissionCurrentIndex();

    std::lock_guard<std::mutex> guard(MUTEXMissions);
    mapMissions.find(key)->second.setActiveIndex(index);
}

bool MaceData::getMissionList(const int &systemID, const MissionItem::MISSIONTYPE &type, const MissionItem::MISSIONSTATE &state, MissionItem::MissionList &missionList) const
{
    return false;
    //    bool rtnValue = false;
    //    switch(state)
    //    {
    //    case Data::MissionTypeState::CURRENT:
    //    {
    //        throw std::runtime_error("MaceData is not currently supporting get mission lists from the current queue");
    //        break;
    //    }
    //    case Data::MissionTypeState::ONBOARD:
    //    {
    //        std::lock_guard<std::mutex> guard(MUTEXMissions);
    //        if(mapOnboardMissions.count(systemID))
    //        {
    //            //this implies there is atleast a mission associated with the requested vehicleID
    //            std::map<MissionItem::MISSIONTYPE,MissionItem::MissionList>::iterator it;
    //            std::map<MissionItem::MISSIONTYPE,MissionItem::MissionList> subList = mapOnboardMissions.at(systemID);

    //            //it = mapGenericMissions.at(missionKey.m_systemID).find(missionKey); I dont understand why I cannot do this
    //            it = subList.find(type);
    //            if(it != mapOnboardMissions.at(systemID).end())
    //            {
    //                //this implies that the iterator now points to the missionList
    //                missionList = it->second;
    //                rtnValue = true;
    //            }
    //        }
    //        break;
    //    }
    //    default:
    //    {
    //        throw std::runtime_error("MaceData has been asked to get a mission based on an unrecognized state");
    //        break;
    //    }
    //    }
    //    return rtnValue;
}

bool MaceData::getMissionList(const MissionItem::MissionKey &missionKey, MissionItem::MissionList &missionList) const
{
    //this will search through the proposed mission queue for the mission key
    //and return the list associated with this
    std::map<MissionItem::MissionKey,MissionItem::MissionList>::const_iterator it;
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    it = mapMissions.find(missionKey);
    if(it != mapMissions.end())
    {
        missionList = it->second;
        return true;
    }
    return false;
}

bool MaceData::getCurrentMissionKey(const int &systemID, MissionItem::MissionKey &key) const
{
    bool returnVal = true;
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    try{
        key = mapCurrentMission.at(systemID);
    }catch(const std::out_of_range &oor){
        std::cout<<"getCurrentMissionKey tried to access an item OOR"<<std::endl;
        returnVal = false;
    }
    return returnVal;
}

bool MaceData::getCurrentMission(const int &systemID, MissionItem::MissionList &cpyMission) const
{
    bool returnVal = true;
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    try{
        MissionItem::MissionKey key = mapCurrentMission.at(systemID);
        cpyMission = mapMissions.at(key);
    }catch(const std::out_of_range &oor){
        std::cout<<"getCurrentMission tried to access an item OOR"<<std::endl;
        returnVal = false;
    }
    return returnVal;
}

bool MaceData::getCurrentMissionValidity(const int &systemID) const
{
    bool returnVal = true;
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    try{
        MissionItem::MissionKey key = mapCurrentMission.at(systemID);
    }catch(const std::out_of_range &oor){
        std::cout<<"getCurrentMission tried to access an item OOR"<<std::endl;
        returnVal = false;
    }
    return returnVal;
}

bool MaceData::getMissionKeyValidity(const MissionItem::MissionKey &key) const
{
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    if(mapMissions.count(key) > 0)
        return true;
}
/*
The following methods aid getting the mission list from the mace data class. The following methods aid getting
the current mission object and keys.
*/

std::vector<MissionItem::MissionKey> MaceData::getOnboardMissionKeys(const int &systemID)
{
    std::vector<MissionItem::MissionKey> keyVector;
    //loop through the onboard missions

    std::lock_guard<std::mutex> guard(MUTEXMissions);
    std::map<MissionItem::MissionKey,MissionItem::MissionList>::iterator it;

    for (it=mapMissions.begin(); it!=mapMissions.end(); ++it)
    {
        MissionItem::MissionList list;
        MissionItem::MissionKey key = it->first;
        if((key.m_systemID == systemID) && (it->second.getMissionTXState() == MissionItem::MISSIONSTATE::ONBOARD))
            keyVector.push_back(key);
    }
    return keyVector;
}

void MaceData::removeFromMissionMap(const MissionItem::MissionKey &missionKey)
{
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    mapMissions.erase(missionKey);
}

MissionItem::MissionKey MaceData::receivedMissionACKKey(const MissionItem::MissionKey &missionKey, const MissionItem::MISSIONSTATE &newState)
{
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    try{
        MissionItem::MissionList copyList = mapMissions.at(missionKey);
        MissionItem::MissionKey newKey = missionKey;
        newKey.m_missionState = newState;
        copyList.setMissionKey(newKey);
        mapMissions.erase(missionKey);
        mapMissions[newKey] = copyList;
        return newKey;
    }catch(const std::out_of_range &oor){
        std::cout<<"receivedMissionACKKey tried to access an item OOR"<<std::endl;
    }
    return missionKey;
}

void MaceData::receivedNewMission(const MissionItem::MissionList &missionList)
{
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    MissionItem::MissionKey key = missionList.getMissionKey();
    mapMissions[key] = missionList;

    //KEN fix: we should update this current mission list, the issue is the lock ga
    //    if(missionList.getMissionTXState() == MissionItem::MISSIONSTATE::CURRENT)
    //        this->updateCurrentMission(missionList.getMissionKey());
}

/*
The following methods update the mission type state of the appropriate mission items.
*/

void MaceData::updateMissionExeState(const MissionItem::MissionKey &missionKey, const Data::MissionExecutionState &state)
{
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    std::map<MissionItem::MissionKey, MissionItem::MissionList>::iterator it;
    it = mapMissions.find(missionKey);
    if(it != mapMissions.end())
    {
        //this implies we have knowledge about this vehicle
        MissionItem::MissionList mission = it->second;
        mission.setMissionExeState(state);
    }
}

bool MaceData::updateOnboardMission(const MissionItem::MissionKey &missionKey)
{
    int systemID = missionKey.m_systemID;
    MissionItem::MISSIONTYPE type = missionKey.m_missionType;

    std::lock_guard<std::mutex> guard(MUTEXMissions);

    std::map<MissionItem::MissionKey, MissionItem::MissionList>::iterator it;
    it = mapMissions.find(missionKey);

    if(it != mapMissions.end())
    {
        //this implies we have knowledge about this vehicle
        MissionItem::MissionList mission = it->second;
        mission.setMissionTXState(MissionItem::MISSIONSTATE::ONBOARD);
        mapMissions[missionKey] = mission;
        return true;
    }
    return false;
}

bool MaceData::checkForCurrentMission(const MissionItem::MissionKey &missionKey)
{
    int systemID = missionKey.m_systemID;
    std::lock_guard<std::mutex> guard(MUTEXMissions);

    if((missionKey.m_missionState == MissionItem::MISSIONSTATE::CURRENT) &&
            ((missionKey.m_missionType == MissionItem::MISSIONTYPE::AUTO) ||
             (missionKey.m_missionType == MissionItem::MISSIONTYPE::GUIDED)))
    {
        //let us remove the old guided mission because this will no longer be valid
        if(mapCurrentMission.count(systemID))
        {
            MissionItem::MissionKey oldKey = mapCurrentMission.at(systemID);
            try{
                if(mapMissions.at(oldKey).getMissionType() == MissionItem::MISSIONTYPE::GUIDED)
                    mapMissions.erase(oldKey);
            }catch(const std::out_of_range &oor){
                std::cout<<"checkForCurrentMission tried to access an item OOR"<<std::endl;
            }
        }
        mapCurrentMission[systemID] = missionKey;
        return true;
    }
    return false;
}

/////////////////////////////////////////////////////////
/// PATH PLANNING DATA
/////////////////////////////////////////////////////////

octomap::OcTree MaceData::getOccupancyGrid3D()
{
    std::lock_guard<std::mutex> guard(m_Mutex_OctomapWrapper);
    return m_OccupancyTree;
}

mace::maps::Data2DGrid<mace::maps::OctomapWrapper::OccupiedResult> MaceData::getCompressedOccupancyGrid2D()
{
    std::lock_guard<std::mutex> guard(m_Mutex_OctomapWrapper);
    return m_OccupancyMap;
}

} //end of namespace MaceCore
