#include "mission_list.h"

namespace MissionItem {

MissionList::MissionList()
{

}

void MissionList::insertMissionItem(const std::shared_ptr<AbstractMissionItem> missionItem)
{
    missionQueue.push_back(missionItem);
}

std::shared_ptr<AbstractMissionItem> MissionList::getMissionItem(const int &index)
{
    return missionQueue.at(index);
}

int MissionList::getQueueSize()
{
    return missionQueue.size();
}

}//end of namespace MissionItem
