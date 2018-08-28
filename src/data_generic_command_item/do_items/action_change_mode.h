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
    /**
     * @brief getCommandType
     * @return
     */
    COMMANDITEM getCommandType() const override;

    /**
     * @brief getDescription
     * @return
     */
    std::string getDescription() const override;

    /**
     * @brief hasSpatialInfluence
     * @return
     */
    bool hasSpatialInfluence() const override;

    /**
     * @brief getClone
     * @return
     */
    std::shared_ptr<AbstractCommandItem> getClone() const override;

    /**
     * @brief getClone
     * @param state
     */
    void getClone(std::shared_ptr<AbstractCommandItem> &command) const override;

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
        out<<"Command Change Mode( Mode: "<<obj.vehicleMode<<")";
        return out;
    }

private:
    std::string vehicleMode;

};

} //end of namespace MissionItem

#endif // ACTION_CHANGE_MODE_H
