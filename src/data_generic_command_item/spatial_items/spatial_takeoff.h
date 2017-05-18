#ifndef SPATIAL_TAKEOFF_H
#define SPATIAL_TAKEOFF_H

#include <iostream>

#include "data/command_item_type.h"

#include "data_generic_command_item/abstract_command_item.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace CommandItem {

template <class T>
class SpatialTakeoff : public AbstractCommandItem
{

public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;

public:
    SpatialTakeoff();
    SpatialTakeoff(const SpatialTakeoff &obj);
    SpatialTakeoff(const int &systemOrigin, const int &systemTarget = 0);

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
        AbstractCommandItem::operator =(rhs);
        this->position = rhs.position;
        this->positionFlag = rhs.positionFlag;
    }

    bool operator == (const SpatialTakeoff &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
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
