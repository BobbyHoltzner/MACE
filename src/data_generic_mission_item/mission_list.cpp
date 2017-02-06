#include "mission_list.h"

namespace MissionItem {

MissionList::MissionList()
{

}

void MissionList::insertMissionItem(AbstractMissionItem *missionItem)
{
    missionQueue.push_back(missionItem);
}

}//end of namespace MissionItem
