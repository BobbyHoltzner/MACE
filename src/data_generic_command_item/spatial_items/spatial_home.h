#ifndef SPATIAL_HOME_H
#define SPATIAL_HOME_H

#include <iostream>

#include "data/command_item_type.h"
#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_state_item/state_generic_position.h"


namespace CommandItem {

template<class T>
class SpatialHome : public AbstractCommandItem, public DataState::StateGenericPosition<T>
{
public:
    SpatialHome();
    SpatialHome(const SpatialHome &obj);
    SpatialHome(const int &systemOrigin, const int &systemTarget = 0);

public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;


public:
    void operator = (const SpatialHome &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        DataState::StateGenericPosition<T>::operator =(rhs);
    }

    bool operator == (const SpatialHome &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(!DataState::StateGenericPosition<T>::operator ==(rhs))
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialHome &rhs) {
        return !(*this == rhs);
    }
};

} //end of namespace MissionItem

#endif // SPATIAL_HOME_H
