#ifndef ACTION_CHANGE_MODE_H
#define ACTION_CHANGE_MODE_H

#include <iostream>
#include <string>

#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/abstract_command_item.h"


namespace CommandItem {

class ActionChangeMode : public AbstractCommandItem
{
public:
    virtual COMMANDITEM getCommandType() const;

    virtual std::string getDescription() const;

    virtual bool hasSpatialInfluence() const;

public:
    ActionChangeMode();
    ActionChangeMode(const ActionChangeMode &obj);
    ActionChangeMode(const int &systemOrigin, const int &systemTarget);

public:
    void setRequestMode(const std::string &mode)
    {
        vehicleMode = mode;
    }

    std::string getRequestMode() const{
        return vehicleMode;
    }


public:
    void operator = (const ActionChangeMode &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->vehicleMode = rhs.vehicleMode;
    }

    bool operator == (const ActionChangeMode &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->vehicleMode != rhs.vehicleMode){
            return false;
        }
        return true;
    }

    bool operator != (const ActionChangeMode &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream &operator<<(std::ostream &out, const ActionChangeMode &obj)
    {
        out<<"Command Change Mode( Target ID: "<<obj.targetSystem<<", Generating ID: "<<obj.originatingSystem<<", Mode: "<<obj.vehicleMode<<")";
        return out;
    }

private:
    std::string vehicleMode;

};

} //end of namespace MissionItem

#endif // ACTION_CHANGE_MODE_H
