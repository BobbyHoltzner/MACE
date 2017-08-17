#ifndef ACTION_ARM_H
#define ACTION_ARM_H

#include <iostream>

#include "command_item_type.h"
#include "data_generic_command_item/abstract_command_item.h"


namespace CommandItem {

class ActionArm : public AbstractCommandItem
{
public:
    virtual COMMANDITEM getCommandType() const;

    virtual std::string getDescription() const;

    virtual bool hasSpatialInfluence() const;

public:
    ActionArm();
    ActionArm(const ActionArm &obj);
    ActionArm(const int &systemOrigin, const int &targetSystem);

public:
    void setVehicleArm(const bool &arm)
    {
        actionArm = arm;
    }

    bool getRequestArm() const{
        return actionArm;
    }

public:
    void operator = (const ActionArm &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->actionArm = rhs.actionArm;
    }

    bool operator == (const ActionArm &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->actionArm != rhs.actionArm){
            return false;
        }
        return true;
    }

    bool operator != (const ActionArm &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream &operator<<(std::ostream &out, const ActionArm &obj)
    {
        out<<"Command Arm( Target ID: "<<obj.targetSystem<<", Generating ID: "<<obj.originatingSystem<<", Request arm: "<<obj.actionArm<<")";
        return out;
    }


private:
    bool actionArm;

};

} //end of namespace MissionItem

#endif // ACTION_ARM_H
