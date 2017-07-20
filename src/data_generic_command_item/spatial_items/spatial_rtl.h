#ifndef SPATIAL_RTL_H
#define SPATIAL_RTL_H

#include <iostream>

#include "data/command_item_type.h"

#include "data_generic_command_item/abstract_command_item.h"

namespace CommandItem {

class SpatialRTL : public AbstractCommandItem
{
public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;

public:
    SpatialRTL();
    SpatialRTL(const SpatialRTL &obj);
    SpatialRTL(const int &originatingSystem, const int &systemTarget = 0);

public:
    void operator = (const SpatialRTL &rhs)
    {
        AbstractCommandItem::operator =(rhs);
    }

    bool operator == (const SpatialRTL &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
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
