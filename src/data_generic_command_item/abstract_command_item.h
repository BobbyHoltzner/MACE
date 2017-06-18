#ifndef ABSTRACT_MISSION_ITEM_H
#define ABSTRACT_MISSION_ITEM_H

#include <string>

#include "data/command_item_type.h"
#include "data/coordinate_frame.h"

namespace CommandItem {

class AbstractCommandItem
{
protected:
    AbstractCommandItem():
        originatingSystem(0), targetSystem(0)
    {

    }

    AbstractCommandItem(const int &systemOrigin, const int &systemTarget = 0):
        originatingSystem(systemOrigin), targetSystem(systemTarget)
    {

    }

public:

    virtual Data::CommandItemType getCommandType() const = 0;

    virtual bool hasSpatialInfluence() const = 0;

    virtual std::string getDescription() const = 0;

public:
    void setTargetSystem(const int &systemID){
        targetSystem = systemID;
    }

    int getTargetSystem() const{
        return targetSystem;
    }

    void setGeneratingSystem(const int &systemID){
        originatingSystem = systemID;
    }

    int getGeneratingSystem() const{
        return originatingSystem;
    }

public:
    void operator = (const AbstractCommandItem &rhs)
    {
        this->targetSystem = rhs.targetSystem;
        this->originatingSystem = rhs.originatingSystem;
    }

    bool operator == (const AbstractCommandItem &rhs) {
        if(this->targetSystem != rhs.targetSystem){
            return false;
        }
        if(this->originatingSystem != rhs.originatingSystem){
            return false;
        }
        return true;
    }

    bool operator != (const AbstractCommandItem &rhs) {
        return !(*this == rhs);
    }

protected:
    int originatingSystem;
    int targetSystem;
};

}
#endif // ABSTRACT_MISSION_ITEM_H
