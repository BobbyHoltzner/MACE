#ifndef ABSTRACT_MISSION_ITEM_H
#define ABSTRACT_MISSION_ITEM_H

#include <string>

#include "mission_item_types.h"

namespace MissionItem {

class AbstractMissionItem
{
public:

    virtual MissionItemType getMissionType() const = 0;

    virtual bool hasSpatialMissionInfluence() const = 0;

    virtual std::string getDescription() const = 0;

};


}
#endif // ABSTRACT_MISSION_ITEM_H
