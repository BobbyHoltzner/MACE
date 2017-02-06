#ifndef MISSION_LIST_H
#define MISSION_LIST_H

#include <list>
#include "abstract_mission_item.h"

namespace MissionItem {

class MissionList
{
public:
    MissionList();

    void insertMissionItem(AbstractMissionItem* missionItem);
    int getQueueSize();

public:
    std::list<AbstractMissionItem*> missionQueue;
};

} //end of namespace MissionItem
#endif // MISSION_LIST_H
