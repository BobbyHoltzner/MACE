#ifndef MISSION_LIST_H
#define MISSION_LIST_H

#include <memory>
#include <vector>
#include "abstract_mission_item.h"

namespace MissionItem {

class MissionList
{
public:
    MissionList();
    void initializeQueue(const int &size);
    void insertMissionItem(const std::shared_ptr<AbstractMissionItem> missionItem);
    void replaceMissionItemAtIndex(const std::shared_ptr<AbstractMissionItem> missionItem, const int &index);

    std::shared_ptr<AbstractMissionItem> getMissionItem(const int &index);

    int getQueueSize();

public:

    std::vector<std::shared_ptr<AbstractMissionItem>> missionQueue;
};

} //end of namespace MissionItem
#endif // MISSION_LIST_H
