#ifndef SPATIAL_TAKEOFF_H
#define SPATIAL_TAKEOFF_H

#include <iostream>

#include "data/mission_item_type.h"

#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"



namespace MissionItem {

template <class T>
class SpatialTakeoff : public AbstractMissionItem
{
public:
    SpatialTakeoff();

    virtual Data::MissionItemType getMissionType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialMissionInfluence()const;

public:
    bool getPositionFlag() const {
        return positionFlag;
    }

    void setPositionFlag(const bool &flag){
        this->positionFlag = flag;
    }

public:
    void operator = (const SpatialTakeoff &rhs)
    {
        AbstractMissionItem::operator =(rhs);
        this->position = rhs.position;
        this->positionFlag = rhs.positionFlag;
    }

    bool operator == (const SpatialTakeoff &rhs) {
        if(!AbstractMissionItem::operator ==(rhs))
        {
            return false;
        }
        if(this->position != rhs.position){
            return false;
        }
        if(this->positionFlag != rhs.positionFlag){
            return false;
        }
        return true;
    }

    bool operator != (const SpatialTakeoff &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out);


    //KEN FIX FLAG: This should really be based on how we
    //handle the position element in the future. Basically
    //if the position is assigned, then switch the flag
    //one way or another
private:
    bool positionFlag; //True says takeoff position was set

public:
    T position;
};

} //end of namespace MissionItem

#endif // SPATIAL_TAKEOFF_H
