#ifndef MISSION_ITEM_ACHIEVED_H
#define MISSION_ITEM_ACHIEVED_H

#include "data/mission_key.h"

namespace MissionItem {

class MissionItemAchieved
{
public:
    MissionItemAchieved();
    MissionItemAchieved(const Data::MissionKey &missionKey, const int &index);

public:
    void setMissionKey(const Data::MissionKey &missionKey){
        this->key = missionKey;
    }

    Data::MissionKey getMissionKey() const{
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
    Data::MissionKey key;
    int indexAchieved;
};

} //end of namepsace Missionitem
#endif // MISSION_ITEM_ACHIEVED_H
