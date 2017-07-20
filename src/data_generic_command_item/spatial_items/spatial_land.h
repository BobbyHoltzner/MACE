#ifndef SPATIAL_LAND_H
#define SPATIAL_LAND_H

#include <iostream>

#include "data/command_item_type.h"

#include "data_generic_command_item/abstract_command_item.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace CommandItem {

template <class T>
class SpatialLand : public AbstractCommandItem
{
public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;

public:
    SpatialLand();
    SpatialLand(const SpatialLand &obj);
    SpatialLand(const int &systemOrigin, const int &systemTarget = 0);

    bool getLandFlag() const;
    void setLandFlag(const bool &landFlag);

public:
    void operator = (const SpatialLand &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->position = rhs.position;
    }

    bool operator == (const SpatialLand &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
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

//    template(typename DataState::StateGlobalPosition)
//    friend std::ostream &operator<<(std::ostream &out, const SpatialLand<DataState::StateGlobalPosition> &obj)
//    {
//        out<<"Spatial Global Land( Target ID: "<<obj.targetSystem<<", Generating ID: "<<obj.originatingSystem<<", Latitude: "<<obj.position.latitude<<", Longitude: "<<obj.position.longitude<<", Altitude: "<<obj.position.altitude<<")";
//        return out;
//    }

//    friend std::ostream &operator<<(std::ostream &out, const SpatialLand<DataState::StateGlobalPosition> &obj)
//    {
//        out<<"Spatial Global Land( Target ID: "<<obj.targetSystem<<", Generating ID: "<<obj.originatingSystem<<", Latitude: "<<obj.position.latitude<<", Longitude: "<<obj.position.longitude<<", Altitude: "<<obj.position.altitude<<")";
//        return out;
//    }

public:
    T position;

private:
    bool landFlag;

};

} //end of namespace MissionItem

#endif // SPATIAL_LAND_H
