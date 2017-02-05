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
    ActionChangeMode();
    ActionChangeMode(const int &vehicleID, const std::string &mode);

    ActionChangeMode(const ActionChangeMode &actionArm);

public:
    void setRequestMode(const std::string &mode)
    {
        m_CommandVehicleMode = mode;
    }

    std::string getRequestMode(){
        return m_CommandVehicleMode;
    }

    void setVehicleID(const int &vehicleID){
        m_VehicleID = vehicleID;
    }

    int getVehicleID() const{
        return m_VehicleID;
    }

public:
    bool operator == (const ActionChangeMode &rhs) {
        if(this->m_CommandVehicleMode != rhs.m_CommandVehicleMode){
            return false;
        }
        if(this->m_VehicleID != rhs.m_VehicleID){
            return false;
        }
        return true;
    }

    bool operator != (const ActionChangeMode &rhs) {
        return !(*this == rhs);
    }

private:
    int m_VehicleID;
    std::string m_CommandVehicleMode;

};

} //end of namespace MissionItem

#endif // ACTION_CHANGE_MODE_H
