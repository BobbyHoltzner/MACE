#ifndef MISSION_ITEM_ACHIEVED_H
#define MISSION_ITEM_ACHIEVED_H

#include "mission_key.h"

namespace MissionItem {

class MissionItemAchieved
{
public:
    MissionItemAchieved();
    MissionItemAchieved(const MissionKey &missionKey, const int &index);

public:
    void setMissionKey(const MissionKey &missionKey){
        this->key = missionKey;
    }

    MissionKey getMissionKey() const{
        return key;
    }

    void setMissionAchievedIndex(const int &index){
        this->indexAchieved = index;
    }

    int getMissionAchievedIndex() const{
        return indexAchieved;
    }
    void operator = (const MissionItemAchieved &rhs)
    {
        this->key = rhs.key;
        this->indexAchieved = rhs.indexAchieved;
    }

    bool operator == (const MissionItemAchieved &rhs) {
        if(this->key != rhs.key){
            return false;
        }
        if(this->indexAchieved != rhs.indexAchieved){
            return false;
        }
        return true;
    }

    bool operator != (const MissionItemAchieved &rhs) {
        return !(*this == rhs);
    }

protected:
    MissionKey key;
    int indexAchieved;
};

} //end of namepsace Missionitem
#endif // MISSION_ITEM_ACHIEVED_H
