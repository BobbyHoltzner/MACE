#ifndef DYNAMIC_MISSION_QUEUE_H
#define DYNAMIC_MISSION_QUEUE_H

#include "../mission_items/mission_list.h"
#include "dynamic_target_list.h"

namespace TargetItem {

class DynamicMissionQueue
{
public:
    DynamicMissionQueue();

    DynamicMissionQueue(const MissionItem::MissionKey &key, const unsigned int &index);

    DynamicMissionQueue(const DynamicMissionQueue &copy);


public:
    DynamicMissionQueue& operator = (const DynamicMissionQueue &rhs)
    {
        this->missionKey = rhs.missionKey;
        this->describingMissionItem = rhs.describingMissionItem;
        this->m_TargetList = rhs.m_TargetList;
        return *this;
    }

    bool operator == (const DynamicMissionQueue &rhs) const{
        if(this->missionKey != rhs.missionKey){
            return false;
        }
        if(this->describingMissionItem != rhs.describingMissionItem){
            return false;
        }
        if(this->m_TargetList != rhs.m_TargetList){
            return false;
        }
        return true;
    }

    bool operator != (const DynamicMissionQueue &rhs) const{
        return !(*this == rhs);
    }

public:
    MissionItem::MissionKey missionKey;
    unsigned int describingMissionItem = 0;

    TargetItem::DynamicTargetList m_TargetList;
};

} //end of namespace DynamicMissionQueue
#endif // DYNAMIC_MISSION_QUEUE_H
