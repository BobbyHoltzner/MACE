#include "mission_list.h"

namespace MissionItem {

MissionList::MissionList() :
    missionType(Data::MissionType::AUTO_CURRENT), activeMissionItem(0)
{

}
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

void MissionList::replaceMissionQueue(const std::vector<std::shared_ptr<AbstractMissionItem>> &newQueue)
{
    missionQueue.clear();
    missionQueue = newQueue;
}

MissionList::MissionListStatus MissionList::getMissionListStatus() const
{
    std::vector<int> nullItems;
    MissionListState missionState = MissionListState::COMPLETE;

    int index = 0;
    for(std::vector<std::shared_ptr<AbstractMissionItem>>::const_iterator it = missionQueue.begin(); it != missionQueue.end(); ++it) {
        if(!*it)
        {
            //This should see that the value is null
            nullItems.push_back(index);
            missionState = MissionListState::INCOMPLETE;
        }
        index++;
    }

    MissionListStatus missionStatus;
    missionStatus.state = missionState;
    missionStatus.remainingItems = nullItems;

    return missionStatus;
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

int MissionList::getQueueSize() const
{
    return missionQueue.size();
}

int MissionList::getActiveIndex() const
{
    return activeMissionItem;
}

std::shared_ptr<AbstractMissionItem> MissionList::getActiveMissionItem()
{
    return (getMissionItem(getActiveIndex()));
}

void MissionList::setActiveIndex(const int &activeIndex)
{
    activeMissionItem = activeIndex;
}


}//end of namespace MissionItem
