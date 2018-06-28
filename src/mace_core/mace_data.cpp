#include "mace_data.h"

namespace MaceCore{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MISSION METHODS | PUSHING TO MACE DATA
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//!
//! \brief appendAssociatedMissionMap Append a mission list
//! \param missionList Mission list
//! \return Key of new mission list
//!
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

//!
//! \brief appendAssociatedMissionMap Append a mission list with a new system ID
//! \param newSystemID New system ID
//! \param missionList Mission list
//! \return Key of new mission list
//!
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

//!
//! \brief getAvailableMissionID Get mission ID based on mission key
//! \param key Key to query
//! \return Mission ID
//!
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
//!
//! \brief updateCurrentMissionItem Update the current mission item
//! \param current Current mission item
//! \return
//!
bool MaceData::updateCurrentMissionItem(const MissionItem::MissionItemCurrent &current)
{
    MissionItem::MissionKey key = current.getMissionKey();
    int index = current.getMissionCurrentIndex();

    std::lock_guard<std::mutex> guard(MUTEXMissions);
    mapMissions.find(key)->second.setActiveIndex(index);
}

//!
//! \brief getMissionList Get mission list corresponding to the system ID, mission type, mission state
//! \param systemID System ID
//! \param type Mission type
//! \param state Mission state
//! \param missionList Container for the mission list
//! \return True if mission list exists
//!
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

//!
//! \brief getMissionList Get mission list based on mission key
//! \param missionKey Mission key to query
//! \param missionList Container for the mission list
//! \return True if mission list exists
//!
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

//!
//! \brief getCurrentMissionKey Get current mission key for the specified system ID
//! \param systemID System ID to query
//! \param key Container for the mission key
//! \return True if mission key exists
//!
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

//!
//! \brief getCurrentMission Get current mission for the specified system ID
//! \param systemID System ID to query
//! \param cpyMission Container for the mission list
//! \return  True if mission exists
//!
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

//!
//! \brief getCurrentMissionValidity Get current mission validity for the specified system ID
//! \param systemID System ID to query
//! \return True if current mission exists
//!
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

//!
//! \brief getMissionKeyValidity Get current mission key validity
//! \param key Key to query
//! \return True if mission key exists
//!
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
//!
//! \brief getOnboardMissionKeys Get a list of mission keys for the specified system ID
//! \param systemID System ID to query
//! \return Vector of mission keys
//!
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

//!
//! \brief removeFromMissionMap Remove a mission with the corresponding key from the map
//! \param missionKey Mission key to remove
//!
void MaceData::removeFromMissionMap(const MissionItem::MissionKey &missionKey)
{
    std::lock_guard<std::mutex> guard(MUTEXMissions);
    mapMissions.erase(missionKey);
}

//!
//! \brief receivedMissionACKKey Handle a received mission ACK key
//! \param key Key of the mission
//! \param newState New misison state
//! \return Key for the current mission in its current state
//!
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

//!
//! \brief receivedNewMission Received the full mission
//! \param missionList Received mission list
//!
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

//!
//! \brief updateMissionExeState Update the execution state of the mission with the corresponding key
//! \param missionKey Mission key to update
//! \param state New execution state
//!
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

//!
//! \brief updateOnboardMission Update the onboard mission with the new mission key
//! \param missionKey New mission key
//! \return True if update successful
//!
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

//!
//! \brief checkForCurrentMission Check for the current mission corresponding to the key
//! \param missionKey Mission key to query
//! \return True if current mission exists
//!
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
//!
//! \brief getOctomapDimensions Get the current octomap dimensions
//! \param minX Minimum x value of the bounding box
//! \param maxX Maximum x value of the bounding box
//! \param minY Minimum y value of the bounding box
//! \param maxY Maximum y value of the bounding box
//! \param minZ Minimum z value of the bounding box
//! \param maxZ Maximum z value of the bounding box
//!
void MaceData::getOctomapDimensions(double &minX, double &maxX, double &minY, double &maxY, double &minZ, double &maxZ) const
{
    std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
    m_OctomapWrapper->getTreeDimensions(minX,maxX,minY,maxY,minZ,maxZ);
}

//!
//! \brief updateOctomapProperties Update the octomap properties
//! \param properties New octomap properties
//! \return True if successful
//!
bool MaceData::updateOctomapProperties(const mace::maps::OctomapSensorDefinition &properties)
{
    std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
    return m_OctomapWrapper->updateSensorProperties(properties);
}

//!
//! \brief updateMappingProjectionProperties Update octomap projection properties
//! \param properties New octomap projection properties
//! \return True if successful
//!
bool MaceData::updateMappingProjectionProperties(const mace::maps::Octomap2DProjectionDefinition &properties)
{
    std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
}

//!
//! \brief loadOccupancyEnvironment Load an occupancy map from a file
//! \param filePath File path
//! \return True if successful
//!
bool MaceData::loadOccupancyEnvironment(const string &filePath)
{
    std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
    return m_OctomapWrapper->loadOctreeFromBT(filePath);
}

//!
//! \brief getOccupancyGrid3D Get the current 3D occupancy map
//! \return True if successful
//!
octomap::OcTree MaceData::getOccupancyGrid3D() const
{
    std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
    return *m_OctomapWrapper->get3DOccupancyMap();
}

//!
//! \brief getCompressedOccupancyGrid2D Get the current compressed 2D occupancy map
//! \return True if successful
//!
mace::maps::Data2DGrid<mace::maps::OccupiedResult> MaceData::getCompressedOccupancyGrid2D() const
{
    std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
    return *m_OctomapWrapper->get2DOccupancyMap();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MACE BOUNDARY METHODS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//! \brief updateBoundary Update the boundary list with new boundary list
//! \param boundary Boundary to update (based on boundary list key)
//!
void MaceData::updateBoundary(const BoundaryItem::BoundaryList &boundary)
{
    BoundaryItem::BoundaryKey key = boundary.getBoundaryKey();

    std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
    BoundaryItem::BoundaryMapPair mapPair(key.m_systemID,key.m_boundaryType);
    m_EnvironmentalBoundaryMap[mapPair] = boundary;

    if(key.m_systemID == 0) //we should set this for all of the vehicles
    {

    }
}

//!
//! \brief updateBoundariesNewOrigin
//! \param distance
//! \param bearing
//!
void MaceData::updateBoundariesNewOrigin(const double &distance, const double &bearing)
{
    std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
    std::map<BoundaryItem::BoundaryMapPair,BoundaryItem::BoundaryList>::iterator it = m_EnvironmentalBoundaryMap.begin();

    for(; it!=m_EnvironmentalBoundaryMap.end(); ++it)
    {
        BoundaryItem::BoundaryList list = it->second;
        list.boundingPolygon.applyCoordinateShift(distance, bearing);
    }
}

//!
//! \brief getBoundary Get the boundary based on boundary key
//! \param operationBoundary Container for the operational boundary
//! \param key Boundary key
//! \return True if successful
//!
bool MaceData::getBoundary(BoundaryItem::BoundaryList *operationBoundary, const BoundaryItem::BoundaryKey &key) const
{
    std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
    try{
        BoundaryItem::BoundaryMapPair mapPair(key.m_systemID,key.m_boundaryType);
        *operationBoundary = m_EnvironmentalBoundaryMap.at(mapPair);
    }catch(const std::exception& error)
    {
        std::cout<<"An exception was raised during getBoundary: "<<error.what()<<std::endl;
        operationBoundary = nullptr;
        return false;
    }

    return true;
}

//!
//! \brief getOperationalBoundary Get the operational boundary
//! \param operationBoundary Container for the operational boundary
//! \param vehicleID
//!
void MaceData::getOperationalBoundary(BoundaryItem::BoundaryList* operationBoundary, const int &vehicleID) const
{
    std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
    try{
        BoundaryItem::BoundaryMapPair mapPair(vehicleID,BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);
        *operationBoundary = m_EnvironmentalBoundaryMap.at(mapPair);
    }catch(const std::exception& error)
    {
        std::cout<<"An exception was raised during getOperationalBoundary: "<<error.what()<<std::endl;
        operationBoundary = nullptr;
    }
}

//!
//! \brief getResourceBoundary Get the resource boundary for a specified vehicle
//! \param resourceBoundary Container for the resource boundary
//! \param vehicleID Vehicle ID to grab the resource boundary for
//!
void MaceData::getResourceBoundary(BoundaryItem::BoundaryList* resourceBoundary, const int &vehicleID)
{
    std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
    try{
        BoundaryItem::BoundaryMapPair mapPair(vehicleID,BoundaryItem::BOUNDARYTYPE::RESOURCE_FENCE);
        *resourceBoundary = m_EnvironmentalBoundaryMap.at(mapPair);
    }catch(const std::exception& error)
    {
        std::cout<<"An exception was raised during getResourceBoundary: "<<error.what()<<std::endl;
        resourceBoundary = nullptr;
    }
}


} //end of namespace MaceCore
