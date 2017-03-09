#ifndef SPATIAL_RTL_H
#define SPATIAL_RTL_H

#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_mission_item/mission_item_types.h"


namespace MissionItem {

class SpatialRTL : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialMissionInfluence()const;

public:
    void operator = (const SpatialRTL &rhs)
    {
        AbstractMissionItem::operator =(rhs);
    }

    bool operator == (const SpatialRTL &rhs) {
        if(!AbstractMissionItem::operator ==(rhs))
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialRTL &rhs) {
        return !(*this == rhs);
    }
};

} //end of namespace MissionItem

#endif // SPATIAL_RTL_H