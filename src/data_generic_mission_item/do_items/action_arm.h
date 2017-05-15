#ifndef ACTION_ARM_H
#define ACTION_ARM_H

#include <iostream>

#include "data/mission_item_type.h"
#include "data_generic_mission_item/abstract_mission_item.h"


namespace MissionItem {

class ActionArm : public AbstractMissionItem
{
public:
    virtual Data::MissionItemType getMissionType() const;

    virtual std::string getDescription() const;

    virtual bool hasSpatialMissionInfluence() const;

public:
    ActionArm();
    ActionArm(const int &vehicleID, const bool &arm);

    ActionArm(const ActionArm &actionArm);

public:
    void setVehicleArm(const bool &arm)
    {
        m_ActionArm = arm;
    }

    bool getRequestArm() const{
        return m_ActionArm;
    }

public:
    void operator = (const ActionArm &rhs)
    {
        AbstractMissionItem::operator =(rhs);
        this->m_ActionArm = rhs.m_ActionArm;
    }

    bool operator == (const ActionArm &rhs) {
        if(!AbstractMissionItem::operator ==(rhs))
        {
            return false;
        }
        if(this->m_ActionArm != rhs.m_ActionArm){
            return false;
        }
        return true;
    }

    bool operator != (const ActionArm &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Action Arm(SystemID: "<<m_VehicleID<<", Arm: "<<m_ActionArm<<")";
        return out;
    }

private:
    bool m_ActionArm;

};

} //end of namespace MissionItem

#endif // ACTION_ARM_H
