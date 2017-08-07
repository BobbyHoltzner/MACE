#ifndef MISSION_ITEM_CURRENT_H
#define MISSION_ITEM_CURRENT_H

#include "data/mission_key.h"

namespace MissionItem {

class MissionItemCurrent
{
public:
    MissionItemCurrent();
    MissionItemCurrent(const Data::MissionKey &missionKey, const int &index);

public:
    void setMissionKey(const Data::MissionKey &missionKey){
        this->key = missionKey;
    }

    Data::MissionKey getMissionKey() const{
        return key;
    }

    void setMissionCurrentIndex(const int &index){
        this->indexCurrent = index;
    }

    int getMissionCurrentIndex() const{
        return indexCurrent;
    }
    void operator = (const MissionItemCurrent &rhs)
    {
        this->key = rhs.key;
        this->indexCurrent = rhs.indexCurrent;
    }

    bool operator == (const MissionItemCurrent &rhs) {
        if(this->key != rhs.key){
            return false;
        }
        if(this->indexCurrent != rhs.indexCurrent){
            return false;
        }
        return true;
    }

    bool operator != (const MissionItemCurrent &rhs) {
        return !(*this == rhs);
    }

protected:
    Data::MissionKey key;
    int indexCurrent;
};

} //end of namepsace MissionItem
#endif // MISSION_ITEM_CURRENT_H
