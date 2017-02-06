#include "mission_list.h"

namespace MissionItem {

MissionList::MissionList()
{

}

void MissionList::insertMissionItem(const std::shared_ptr<AbstractMissionItem> missionItem)
{
    missionQueue.push_back(missionItem);
}

int MissionList::getQueueSize()
{
    return missionQueue.size();
}

}//end of namespace MissionItem
