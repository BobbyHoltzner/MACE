#ifndef SPATIAL_WAYPOINT_H
#define SPATIAL_WAYPOINT_H

#include <iostream>

#include "data/command_item_type.h"

#include "data_generic_command_item/abstract_command_item.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

#include "data_generic_state_item/state_generic_position.h"

namespace CommandItem {

template <class T>
class SpatialWaypoint : public AbstractCommandItem, public DataState::StateGenericPosition<T>
{
public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;

public:
    SpatialWaypoint();
    SpatialWaypoint(const SpatialWaypoint &obj);
    SpatialWaypoint(const int &systemOrigin, const int &systemTarget = 0);

public:
    void operator = (const SpatialWaypoint &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        DataState::StateGenericPosition<T>::operator =(rhs);
    }

    bool operator == (const SpatialWaypoint &rhs) {
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

    bool operator != (const SpatialWaypoint &rhs) {
        return !(*this == rhs);
    }

};

} //end of namespace MissionItem
#endif // SPATIAL_WAYPOINT_H
