#include "mission_list.h"

namespace MissionItem {

void MissionList::initializeQueue(const int &size)
{
    missionQueue.clear();
    std::vector<std::shared_ptr<AbstractMissionItem>> tmpVector(size,NULL);
    missionQueue = tmpVector;
}

void MissionList::clearQueue()
{
    missionQueue.clear();
}

void MissionList::insertMissionItem(const std::shared_ptr<AbstractMissionItem> missionItem)
{
    missionQueue.push_back(missionItem);
}

void MissionList::replaceMissionItemAtIndex(const std::shared_ptr<AbstractMissionItem> missionItem, const int &index)
{
    missionQueue[index] = missionItem;
}

std::shared_ptr<AbstractMissionItem> MissionList::getMissionItem(const int &index)
{
    return missionQueue[index];
}

int MissionList::getQueueSize()
{
    return missionQueue.size();
}


}//end of namespace MissionItem
