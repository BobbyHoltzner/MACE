#include "mission_key.h"

namespace Data {

MissionKey::MissionKey():
    m_systemID(0),m_creatorID(0),m_missionID(0),m_missionType(Data::MissionType::AUTO),m_missionState(Data::MissionTXState::CURRENT)
{

}

MissionKey::MissionKey(const int &systemID, const int &creatorID, const int &missionID, const Data::MissionType &missionType):
    m_systemID(systemID),m_creatorID(creatorID),m_missionID(missionID),m_missionType(missionType),m_missionState(Data::MissionTXState::CURRENT)
{

}

MissionKey::MissionKey(const int &systemID, const int &creatorID, const int &missionID, const Data::MissionType &missionType, const Data::MissionTXState &missionState):
    m_systemID(systemID),m_creatorID(creatorID),m_missionID(missionID),m_missionType(missionType),m_missionState(missionState)
{

}

MissionKey::MissionKey(const MissionKey &obj)
{
    this->m_systemID = obj.m_systemID;
    this->m_creatorID =obj.m_creatorID;
    this->m_missionID = obj.m_missionID;
    this->m_missionType = obj.m_missionType;
    this->m_missionState = obj.m_missionState;

}

void MissionKey::operator =(const MissionKey &rhs)
{
    this->m_systemID = rhs.m_systemID;
    this->m_creatorID =rhs.m_creatorID;
    this->m_missionID = rhs.m_missionID;
    this->m_missionType = rhs.m_missionType;
    this->m_missionState = rhs.m_missionState;
}

bool MissionKey::operator <(const MissionKey &rhs) const
{
    if(*this == rhs)
        return false;

    if(this->m_systemID > rhs.m_systemID)
        return false;
    else if(this->m_systemID < rhs.m_systemID)
        return true;
    else{ //this implies that targetID was equal
        if(this->m_missionID > rhs.m_missionID)
            return false;
        else if(this->m_missionID < rhs.m_missionID)
            return true;
        else{ //this implies that the missionID was equal
            if(this->m_creatorID > rhs.m_creatorID)
                return false;
            else if(this->m_creatorID < rhs.m_creatorID)
                return true;
            else{ //this implies that the creatorID was equal
                if(this->m_missionType > rhs.m_missionType)
                    return false;
                else if(this->m_missionType < rhs.m_missionType)
                    return true;
                else{ //this implies that the missionType was equal
                    if(this->m_missionState > rhs.m_missionState)
                        return false;
                    else if(this->m_missionState < rhs.m_missionState)
                        return true;
                    else{
                        return false;
                    }
                }
            }
        }
    }

}

bool MissionKey::operator ==(const MissionKey &rhs) const
{
    if(this->m_systemID != rhs.m_systemID)
        return false;
    if(this->m_creatorID != rhs.m_creatorID)
        return false;
    if(this->m_missionID != rhs.m_missionID)
        return false;
    if(this->m_missionType != rhs.m_missionType)
        return false;
    if(this->m_missionState != rhs.m_missionState)
        return false;
    return true;
}

bool MissionKey::operator !=(const MissionKey &rhs) const
{
    return !((*this) == rhs);
}

std::ostream& operator<<(std::ostream& os, const MissionKey& t)
{
    std::stringstream stream;
    stream << std::fixed << "Mission Key: System ID " << std::to_string(t.m_systemID)
           << ", Creator ID"<< std::to_string(t.m_creatorID)
           << ", Mission ID " << std::to_string(t.m_missionID) << ".";
    os << stream.str();

    return os;
}

} //end of namespace Data
