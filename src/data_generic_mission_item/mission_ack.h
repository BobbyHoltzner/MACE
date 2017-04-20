#ifndef MISSION_ACK_H
#define MISSION_ACK_H

#include "data/mission_type.h"

namespace MissionItem {

class MissionACK
{
public:
    MissionACK(const int &systemID);

public:
    void setSystemID(const int &systemID){
        m_SystemID = systemID;
    }

    int getSystemID() const{
        return m_SystemID;
    }

    void setMissionType(const Data::MissionType &missionType){
        this->missionType = missionType;
    }

    Data::MissionType getMissionType() const{
        return missionType;
    }

public:
    void operator = (const MissionACK &rhs)
    {
        this->m_SystemID = rhs.m_SystemID;
        this->missionType = rhs.missionType;
    }

    bool operator == (const MissionACK &rhs) {
        if(this->m_SystemID != rhs.m_SystemID){
            return false;
        }
        if(this->missionType != rhs.missionType){
            return false;
        }
        return true;
    }

    bool operator != (const MissionACK &rhs) {
        return !(*this == rhs);
    }


private:

    //!
    //! \brief m_SystemID
    //!
    int m_SystemID;

    //!
    //! \brief missionType This denotes the queue in which the information should be stored.
    //! This parameter will be packaged in the COMPID protocol for now.
    //!
    Data::MissionType missionType;
};

} //end of namespace MissionItem
#endif // MISSION_ACK_H
