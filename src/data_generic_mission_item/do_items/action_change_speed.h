#ifndef ACTION_CHANGE_SPEED_H
#define ACTION_CHANGE_SPEED_H

#include <iostream>

#include "data/speed_frame.h"

#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_mission_item/mission_item_types.h"

namespace MissionItem {

class ActionChangeSpeed : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType() const;

    virtual std::string getDescription() const;

    virtual bool hasSpatialMissionInfluence() const;

public:
    ActionChangeSpeed();
    ActionChangeSpeed(const int &vehicleID);

public:
    void setSpeedFrame(const Data::SpeedFrame &frame)
    {
        speedFrame = frame;
    }

    Data::SpeedFrame getSpeedFrame() const
    {
        return speedFrame;
    }

    void setDesiredSpeed(const double &speed)
    {
        desiredSpeed = speed;
    }

    double getDesiredSpeed() const
    {
        return desiredSpeed;
    }

public:
    void operator = (const ActionChangeSpeed &rhs)
    {
        AbstractMissionItem::operator =(rhs);
        this->speedFrame = rhs.speedFrame;
        this->desiredSpeed = rhs.desiredSpeed;
    }

    bool operator == (const ActionChangeSpeed &rhs) {
        if(!AbstractMissionItem::operator ==(rhs))
        {
            return false;
        }
        if(this->speedFrame != rhs.speedFrame){
            return false;
        }
        if(this->desiredSpeed != rhs.desiredSpeed){
            return false;
        }
        return true;
    }

    bool operator != (const ActionChangeSpeed &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Action Change Speed(SystemID: "<<m_VehicleID<<", Frame: "<<Data::SpeedFrameToString(speedFrame)<<", Speed: "<<desiredSpeed<<")";
        return out;
    }

private:
    Data::SpeedFrame speedFrame;
    double desiredSpeed;

};

} //end of namespace MissionItem
#endif // ACTION_CHANGE_SPEED_H
