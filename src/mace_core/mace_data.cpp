#include "mace_data.h"

namespace MaceCore{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Current Missions: There are two types of missions associated in these variables. First, onboard missions
/// represent items that have been confirmed at the last instance this mace communicated with the vehicle to
/// be onboard the mace instance associated with the appropriate aircraft. This implies that any mode changes
/// could potentially activate one of these missions. The current mission is the single container holder for
/// what is actively being puruited by the vehicle.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MaceData::getCurrentMission(const Data::MissionKey &missionKey, MissionItem::MissionList &cpyMission)
{
    bool returnVal = true;
    int systemID = missionKey.m_systemID;

    std::lock_guard<std::mutex> guard(MUTEXCurrentMissions);
    try{
        Data::MissionKey tmpKey = mapCurrentMission.at(systemID).getMissionKey();
        if(tmpKey == missionKey)
            cpyMission = mapCurrentMission.at(systemID);
        else
            returnVal = false;
    }catch(const std::out_of_range &oor){
        std::cout<<"getCurrentMission (int,MissionItem::MissionList) tried to access an item OOR"<<std::endl;
        returnVal = false;
    }
    return returnVal;
}

bool MaceData::getCurrentMission(const int &systemID, MissionItem::MissionList &cpyMission)
{
    bool returnVal = true;
    std::lock_guard<std::mutex> guard(MUTEXCurrentMissions);
    try{
        cpyMission = mapCurrentMission.at(systemID);
    }catch(const std::out_of_range &oor){
        std::cout<<"getCurrentMission (int,MissionItem::MissionList) tried to access an item OOR"<<std::endl;
        returnVal = false;
    }
    return returnVal;
}

bool MaceData::getOnboardMissions(const int &systemID, std::map<Data::MissionType, MissionItem::MissionList> &cpyMission)
{
    bool returnVal = true;
    std::lock_guard<std::mutex> guard(MUTEXCurrentMissions);
    try{
        cpyMission = mapOnboardMissions.at(systemID);
    }catch(const std::out_of_range &oor){
        std::cout<<"getOnboardMissions tried to access an item OOR"<<std::endl;
        returnVal = false;
    }
    return returnVal;
}

bool MaceData::updateOnboardMissions(const Data::MissionKey &missionKey)
{
    int systemID = missionKey.m_systemID;
    Data::MissionType missionType = missionKey.m_missionType;
    std::lock_guard<std::mutex> guardAvailable(MUTEXGenericMissions);

    if(mapGenericMissions.count(systemID) > 0) //means the system ID was in there
    {
        std::map<Data::MissionKey, MissionItem::MissionList> vehicleList = mapGenericMissions.at(systemID);
        std::map<Data::MissionKey, MissionItem::MissionList>::iterator itSub;
        itSub = vehicleList.find(missionKey);
        if(itSub != vehicleList.end()) //means the missionKey was in there
        {
            //Ken Fix....weird stuff in here
            MissionItem::MissionList newOnboard = itSub->second;
            std::lock_guard<std::mutex> guardCurrent(MUTEXCurrentMissions);
            mapOnboardMissions[systemID][missionType] = newOnboard;
            return true;
        }
    }
    return false;
}

bool MaceData::updateCurrentMission(const Data::MissionKey &missionKey)
{
    int systemID = missionKey.m_systemID;
    std::lock_guard<std::mutex> guardAvailable(MUTEXGenericMissions);
    std::map<int,std::map<Data::MissionKey, MissionItem::MissionList>>::iterator it;
    it = mapGenericMissions.find(systemID);

    if(it != mapGenericMissions.end()) //means the system ID was in there
    {
        std::map<Data::MissionKey, MissionItem::MissionList> vehicleList = it->second;
        std::map<Data::MissionKey, MissionItem::MissionList>::iterator itSub;
        itSub = vehicleList.find(missionKey);
        if(itSub != vehicleList.end()) //means the missionKey was in there
        {
            std::lock_guard<std::mutex> guardCurrent(MUTEXCurrentMissions);
            mapCurrentMission[systemID] = itSub->second;
            return true;
        }
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Associated Missions: These types of missions are associated to an aircraft and have been proposed by
/// someone in the mace architecture. The mission key associated with the mission object would
/// help track who generated the mission and who the mission is for. However, this does not imply that
/// these are the actual missions aboard the vehicle or associated with the MACE instance.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

MissionItem::MissionList MaceData::appenedAssociatedMissionMap(const int &newSystemID, const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList correctedMissionList = missionList;
    correctedMissionList.setVehicleID(newSystemID);
    int updatedMissionID = getAvailableMissionID(missionList.getMissionKey());
    correctedMissionList.setMissionID(updatedMissionID);

    std::lock_guard<std::mutex> guard(MUTEXGenericMissions);
    mapGenericMissions[newSystemID][correctedMissionList.getMissionKey()] = correctedMissionList;
    return correctedMissionList;
}

MissionItem::MissionList MaceData::appenedAssociatedMissionMap(const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList correctedMissionList = missionList;
    int systemID = missionList.getVehicleID();
    int updatedMissionID = getAvailableMissionID(missionList.getMissionKey());
    correctedMissionList.setMissionID(updatedMissionID);

    std::lock_guard<std::mutex> guard(MUTEXGenericMissions);
    mapGenericMissions[correctedMissionList.getVehicleID()][correctedMissionList.getMissionKey()] = correctedMissionList;
    return correctedMissionList;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Generic Mission Methods
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

int MaceData::getAvailableMissionID(const Data::MissionKey &key)
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

void MaceData::updateReceivingMission(const MissionItem::MissionList &missionList)
{
    std::lock_guard<std::mutex> guard(MUTEXGenericMissions);
    Data::MissionKey key = missionList.getMissionKey();
    mapGenericMissions[key.m_systemID][key] = missionList;
}

void MaceData::finishedReceivingMission(const MissionItem::MissionList &missionList)
{

}

bool MaceData::getMissionList(const Data::MissionKey &missionKey, MissionItem::MissionList &missionList) const
{
    //this will search through the proposed mission queue for the mission key
    //and reutrn the list associated with this
    std::lock_guard<std::mutex> guard(MUTEXGenericMissions);
    if(mapGenericMissions.count(missionKey.m_systemID))
    {
        //this implies there is atleast a mission associated with the requested vehicleID
        std::map<Data::MissionKey,MissionItem::MissionList>::iterator it;
        std::map<Data::MissionKey,MissionItem::MissionList> subList = mapGenericMissions.at(missionKey.m_systemID);

        //it = mapGenericMissions.at(missionKey.m_systemID).find(missionKey); I dont understand why I cannot do this
        it = subList.find(missionKey);
        if(it != mapGenericMissions.at(missionKey.m_systemID).end())
        {
            //this implies that the iterator now points to the missionList
            missionList = it->second;
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

void MaceData::updateMissionID(const int &systemID, const int &prevID, const int &newID)
{
    std::lock_guard<std::mutex> guard(MUTEXGenericMissions);
    int delta = newID - prevID;
    try{
        std::map<Data::MissionKey,MissionItem::MissionList> correctedMissionMap;
        std::map<Data::MissionKey,MissionItem::MissionList> availableMissions = mapGenericMissions[systemID];

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
        mapGenericMissions[systemID] = correctedMissionMap;
    }catch(const std::out_of_range &oor){
        std::cout<<"MACEDATA has requested an OOR in updateMissionID of mace_data.cpp"<<std::endl;
    }
}

//bool MaceData::getMissionList(MissionItem::MissionList &newList, const int &systemID, const MissionItem::MissionList::MissionListState &missionState, const Data::MissionType &missionType) const
//{
//    switch(missionState)
//    {
//    case MissionItem::MissionList::COMPLETE:
//    {
//        break;
//    }
//    case MissionItem::MissionList::INCOMPLETE:
//    {
//        std::lock_guard<std::mutex> guard(INCOMPLETEMissionMUTEX);
//        try{
//            newList = m_INCOMPLETEMission.at(systemID).at(missionType);
//        }catch(const std::out_of_range &oor){
//            std::cout<<"MACEDATA has requested an OOR on incomplete mission map"<<std::endl;
//            return false;
//        }
//        break;
//    }
//    }

//    return true;
//}

}
