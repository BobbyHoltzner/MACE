#include "mission_key.h"

namespace Data {

MissionKey::MissionKey():
    m_targetID(0),m_creatorID(0),m_missionID(0),m_missionType(Data::MissionType::AUTO)
{

}

MissionKey::MissionKey(const int &targetID, const int &creatorID, const int &missionID, const Data::MissionType &missionType):
    m_targetID(targetID),m_creatorID(creatorID),m_missionID(missionID),m_missionType(missionType)
{

}

void MissionKey::operator =(const MissionKey &rhs)
{
    this->m_targetID = rhs.m_targetID;
    this->m_creatorID =rhs.m_creatorID;
    this->m_missionID = rhs.m_missionID;
    this->m_missionType = rhs.m_missionType;
}

bool MissionKey::operator <(const MissionKey &rhs) const
{
    if(*this == rhs)
        return false;

    if(this->m_targetID > rhs.m_targetID)
        return false;
    else if(this->m_targetID < rhs.m_targetID)
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
                else{
                    return false;
                }
            }
        }
    }

}

bool MissionKey::operator ==(const MissionKey &rhs) const
{
    if(this->m_targetID != rhs.m_targetID)
        return false;
    if(this->m_creatorID != rhs.m_creatorID)
        return false;
    if(this->m_missionID != rhs.m_missionID)
        return false;
    if(this->m_missionType != rhs.m_missionType)
        return false;
    return true;
}

bool MissionKey::operator !=(const MissionKey &rhs) const
{
    return !((*this) == rhs);
}

} //end of namespace Data
