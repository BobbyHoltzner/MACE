#ifndef ACTION_ARM_H
#define ACTION_ARM_H

#include "../abstract_mission_item.h"
#include "../mission_item_types.h"

namespace MissionItem {

class ActionArm : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType();

    virtual std::string getDescription();

    virtual bool hasSpatialInfluence();

public:
    void setVehicleArm(const bool &arm)
    {
        m_ActionArm = arm;
    }

    bool getRequestMode(){
        return m_ActionArm;
    }

private:
    bool m_ActionArm;

};

} //end of namespace MissionItem

#endif // ACTION_ARM_H
