#ifndef SPATIAL_LAND_H
#define SPATIAL_LAND_H

#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_mission_item/mission_item_types.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace MissionItem {

template <class T>
class SpatialLand : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialMissionInfluence()const;

public:
    SpatialLand();

    bool getLandFlag() const;
    void setLandFlag(const bool &landFlag);

public:
    void operator = (const SpatialLand &rhs)
    {
        AbstractMissionItem::operator =(rhs);
        this->position = rhs.position;
    }

    bool operator == (const SpatialLand &rhs) {
        if(!AbstractMissionItem::operator ==(rhs))
        {
            return false;
        }
        if(this->position != rhs.position){
            return false;
        }
        return true;
    }

    bool operator != (const SpatialLand &rhs) {
        return !(*this == rhs);
    }

public:
    T position;

private:
    bool landFlag;

};

} //end of namespace MissionItem

#endif // SPATIAL_LAND_H
