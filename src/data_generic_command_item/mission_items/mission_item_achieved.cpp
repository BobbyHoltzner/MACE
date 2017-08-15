#include "mission_item_achieved.h"

namespace MissionItem {

MissionItemAchieved::MissionItemAchieved()
{

}

MissionItemAchieved::MissionItemAchieved(const MissionKey &missionKey, const int &index):
    key(missionKey), indexAchieved(index)
{

}

} //end of namespace MissionItem
