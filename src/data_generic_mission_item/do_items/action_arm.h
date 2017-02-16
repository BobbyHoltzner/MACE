#ifndef ACTION_ARM_H
#define ACTION_ARM_H

#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_mission_item/mission_item_types.h"

namespace MissionItem {

class ActionArm : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType() const;

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

    bool getRequestArm(){
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

private:
    bool m_ActionArm;

};

} //end of namespace MissionItem

#endif // ACTION_ARM_H
