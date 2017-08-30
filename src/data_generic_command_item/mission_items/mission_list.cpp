#include "mission_list.h"

#include <exception>

namespace MissionItem {

MissionList::MissionList() :
    missionKey(0,0,0,MISSIONTYPE::AUTO,MISSIONSTATE::CURRENT),missionExeState(Data::MissionExecutionState::MESTATE_UNEXECUTED),activeMissionItem(0)
{

}

MissionList::MissionList(const int &targetID, const int &generatorID, const MISSIONTYPE &missionType, const MISSIONSTATE &state) :
    missionKey(targetID,generatorID,0,missionType,state),missionExeState(Data::MissionExecutionState::MESTATE_UNEXECUTED),activeMissionItem(0)
{

}

MissionList::MissionList(const int &targetID, const int &generatorID, const MISSIONTYPE &missionType, const MISSIONSTATE &state, const int &size) :
    missionKey(targetID,generatorID,0,missionType,state),missionExeState(Data::MissionExecutionState::MESTATE_UNEXECUTED),activeMissionItem(0)
{
    initializeQueue(size);
}

MissionList::MissionList(const int &targetID, const int &generatorID, const int &missionID, const MISSIONTYPE &missionType, const MISSIONSTATE &state, const int &size) :
    missionKey(targetID,generatorID,missionID,missionType,state),missionExeState(Data::MissionExecutionState::MESTATE_UNEXECUTED),activeMissionItem(0)
{
    initializeQueue(size);
}

MissionList::MissionList(const MissionList &rhs)
{
    this->missionKey = rhs.missionKey;
    this->missionQueue = rhs.missionQueue;
    this->missionExeState = rhs.missionExeState;
    this->activeMissionItem = rhs.activeMissionItem;
}

void MissionList::initializeQueue(const int &size)
{
    if(size <= 0){
        // TODO-Ken/Pat: Throw a message with exception
        std::cout << "Cannot initialize queue of 0" << std::endl;
        throw std::exception();
    }
    missionQueue.clear();
    std::vector<std::shared_ptr<CommandItem::AbstractCommandItem>> tmpVector(size,NULL);
    missionQueue = tmpVector;
}

void MissionList::clearQueue()
{
    missionQueue.clear();
}

void MissionList::replaceMissionQueue(const std::vector<std::shared_ptr<CommandItem::AbstractCommandItem>> &newQueue)
{
    missionQueue.clear();
    missionQueue = newQueue;
}

MissionList::MissionListStatus MissionList::getMissionListStatus() const
{
    std::vector<int> nullItems;
    MissionListState missionState = MissionListState::COMPLETE;

    int index = 0;
    for(std::vector<std::shared_ptr<CommandItem::AbstractCommandItem>>::const_iterator it = missionQueue.begin(); it != missionQueue.end(); ++it) {
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

void MissionList::insertMissionItem(const std::shared_ptr<CommandItem::AbstractCommandItem> missionItem)
{
    missionQueue.push_back(missionItem);
}

void MissionList::replaceMissionItemAtIndex(const std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const int &index)
{
    missionQueue[index] = missionItem;
}

std::shared_ptr<CommandItem::AbstractCommandItem> MissionList::getMissionItem(const int &index) const
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

std::shared_ptr<CommandItem::AbstractCommandItem> MissionList::getActiveMissionItem()
{
    return (getMissionItem(getActiveIndex()));
}

void MissionList::setActiveIndex(const int &activeIndex)
{
    activeMissionItem = activeIndex;
}

std::ostream& operator<<(std::ostream& os, const MissionList& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Mission List Key" << t.getMissionKey()
           <<", Size: " << std::to_string(t.getQueueSize()) << ".";
    os << stream.str();

    return os;
}

}//end of namespace MissionItem
