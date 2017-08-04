#ifndef MISSION_ACK_H
#define MISSION_ACK_H

#include "data/mission_ack.h"
#include "data/mission_key.h"

namespace MissionItem {

class MissionACK
{
public:
    MissionACK(const int &systemID, const Data::MISSION_RESULT &ack, const Data::MissionKey &key, const Data::MissionTXState &newState);

public:
    int getSystemID() const{
        return this->m_SystemID;
    }

    Data::MISSION_RESULT getMissionResult() const{
        return this->result;
    }

    Data::MissionKey getMissionKey() const{
        return this->refKey;
    }

    Data::MissionTXState getNewMissionState() const{
        return this->newState;
    }

    Data::MissionKey getUpdatedMissionKey() const{
        Data::MissionKey key = getMissionKey();
        key.m_missionState = getNewMissionState();
        return key;
    }

public:
    void operator = (const MissionACK &rhs)
    {
        this->m_SystemID = rhs.m_SystemID;
        this->result = rhs.result;
        this->refKey = rhs.refKey;
        this->newState = rhs.newState;
    }

    bool operator == (const MissionACK &rhs) {
        if(this->m_SystemID != rhs.m_SystemID){
            return false;
        }
        if(this->result != rhs.result){
            return false;
        }
        if(!(this->refKey != rhs.refKey)){
            return false;
        }
        if(this->newState != rhs.newState){
            return false;
        }
        return true;
    }

    bool operator != (const MissionACK &rhs) {
        return !(*this == rhs);
    }


private:
    int m_SystemID;

    Data::MISSION_RESULT result;

    Data::MissionKey refKey;

    Data::MissionTXState newState;
};

} //end of namespace MissionItem
#endif // MISSION_ACK_H
