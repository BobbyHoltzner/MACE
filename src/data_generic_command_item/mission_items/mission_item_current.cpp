#include "mission_item_current.h"

namespace MissionItem {

MissionItemCurrent::MissionItemCurrent()
{

}

MissionItemCurrent::MissionItemCurrent(const MissionKey &missionKey, const int &index):
    key(missionKey), indexCurrent(index)
{

}

} //end of namespace MissionItem
