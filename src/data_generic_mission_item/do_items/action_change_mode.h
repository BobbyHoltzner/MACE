#ifndef ACTION_CHANGE_MODE_H
#define ACTION_CHANGE_MODE_H

#include <iostream>
#include <string>

#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_mission_item/mission_item_types.h"

namespace MissionItem {

class ActionChangeMode : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType() const;

    virtual std::string getDescription() const;

    virtual bool hasSpatialMissionInfluence() const;

public:
    ActionChangeMode();
    ActionChangeMode(const int &vehicleID, const std::string &mode);

    ActionChangeMode(const ActionChangeMode &actionArm);

public:
    void setRequestMode(const std::string &mode)
    {
        m_CommandVehicleMode = mode;
    }

    std::string getRequestMode() const{
        return m_CommandVehicleMode;
    }


public:
    void operator = (const ActionChangeMode &rhs)
    {
        AbstractMissionItem::operator =(rhs);
        this->m_CommandVehicleMode = rhs.m_CommandVehicleMode;
    }

    bool operator == (const ActionChangeMode &rhs) {
        if(!AbstractMissionItem::operator ==(rhs))
        {
            return false;
        }
        if(this->m_CommandVehicleMode != rhs.m_CommandVehicleMode){
            return false;
        }
        return true;
    }

    bool operator != (const ActionChangeMode &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Action Change Mode(SystemID: "<<m_VehicleID<<", Mode: "<<m_CommandVehicleMode<<")";
        return out;
    }

private:
    std::string m_CommandVehicleMode;

};

} //end of namespace MissionItem

#endif // ACTION_CHANGE_MODE_H
