#include "mission_item_current.h"

namespace MissionItem {

MissionItemCurrent::MissionItemCurrent()
{

}

MissionItemCurrent::MissionItemCurrent(const Data::MissionKey &missionKey, const int &index):
    key(missionKey), indexCurrent(index)
{

}

} //end of namespace MissionItem
