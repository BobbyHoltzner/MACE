#ifndef ACTION_CHANGE_MODE_H
#define ACTION_CHANGE_MODE_H

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
    void setRequestMode(const std::string &mode)
    {
        m_CommandVehicleMode = mode;
    }

    std::string getRequestMode(){
        return m_CommandVehicleMode;
    }

private:
    std::string m_CommandVehicleMode;

};

} //end of namespace MissionItem

#endif // ACTION_CHANGE_MODE_H
