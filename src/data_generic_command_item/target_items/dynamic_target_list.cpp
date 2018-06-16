#include "dynamic_target_list.h"

#include <exception>

namespace TargetItem {

DynamicTargetList::DynamicTargetList()
{

}

DynamicTargetList::DynamicTargetList(const DynamicTargetList &rhs)
{
    this->activeTargetItem = rhs.activeTargetItem;
    this->targetList = rhs.targetList;
}

size_t DynamicTargetList::listSize() const
{
    return targetList.size();
}

void DynamicTargetList::clearList()
{
    targetList.clear();
}

void DynamicTargetList::appendDynamicTarget(const DynamicTarget &target, const TargetCompletion &state)
{
    DynamicTargetStorage obj(target,state);
    targetList.push_back(obj);
}

void DynamicTargetList::removeTargetAtIndex(const unsigned int &index)
{
    std::list<DynamicTargetStorage>::iterator it = targetList.begin();
    std::advance(it,index);
    targetList.erase(it);
}

void DynamicTargetList::replaceTargetAtIndex(const unsigned int &index, const DynamicTarget &target, const TargetCompletion &state)
{
    DynamicTargetStorage obj(target,state);
    std::list<DynamicTargetStorage>::iterator it = targetList.begin();
    std::advance(it,index);
    targetList.insert(it,obj);
}

void DynamicTargetList::spliceTargetListAtIndex(const unsigned int &index, const std::list<DynamicTargetStorage> &list)
{
    std::list<DynamicTargetStorage> listCopy = list;
    std::list<DynamicTargetStorage>::iterator it = targetList.begin();
    std::advance(it,index);
    targetList.splice(it,listCopy);
}

const DynamicTargetList::DynamicTargetStorage* DynamicTargetList::getTargetStorageAtIndex(const unsigned int &index) const
{
    std::list<DynamicTargetStorage>::const_iterator it = targetList.begin();
    std::advance(it,index);
    return &(*it);
}

const DynamicTargetList::DynamicTarget* DynamicTargetList::getTargetAtIndex(const unsigned int &index) const
{
    std::list<DynamicTargetStorage>::const_iterator it = targetList.begin();
    std::advance(it,index);
    return &(*it).target;
}

const DynamicTargetList::DynamicTarget* DynamicTargetList::getNextIncomplete() const
{
    std::list<DynamicTargetStorage>::const_iterator it = targetList.begin();
    for (; it != targetList.end(); ++it)
    {
        if((*it).state == TargetCompletion::INCOMPLETE)
            return &(*it).target;
    }
    return nullptr;
}


const DynamicTargetList::DynamicTarget* DynamicTargetList::markCompletionState(const unsigned int &index, const TargetCompletion &state)
{
    std::list<DynamicTargetStorage>::iterator it = targetList.begin();
    std::advance(it,index);
    (*it).state = state;

    for (; it != targetList.end(); ++it)
    {
        if((*it).state == TargetCompletion::INCOMPLETE)
            return &(*it).target;
    }
    return nullptr;
}

std::ostream& operator<<(std::ostream& os, const DynamicTargetList& t)
{
    std::stringstream stream;
    stream.precision(6);

    for (size_t i = 0; i < t.listSize(); i++)
    {
        //print the state inside of here
    }
    os << stream.str();

    return os;
}

}//end of namespace MissionItem
