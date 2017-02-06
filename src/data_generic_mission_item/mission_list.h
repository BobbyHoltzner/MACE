#ifndef MISSION_LIST_H
#define MISSION_LIST_H

#include <memory>
#include <list>
#include "abstract_mission_item.h"

namespace MissionItem {

class MissionList
{
public:
    MissionList();

    void insertMissionItem(const std::shared_ptr<AbstractMissionItem> missionItem);
    int getQueueSize();

public:
    std::list<std::shared_ptr<AbstractMissionItem>> missionQueue;
};

} //end of namespace MissionItem
#endif // MISSION_LIST_H
