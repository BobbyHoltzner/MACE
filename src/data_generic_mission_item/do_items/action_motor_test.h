#ifndef ACTION_MOTOR_TEST_H
#define ACTION_MOTOR_TEST_H

#include <iostream>

#include "data/mission_item_type.h"
#include "data_generic_mission_item/abstract_mission_item.h"


namespace MissionItem {

class ActionMotorTest : public AbstractMissionItem
{
    enum ThrottleChannel{
        PERCENTAGE,
        PWM,
        PASS_THROUGH
    };

public:
    virtual Data::MissionItemType getMissionType() const;

    virtual std::string getDescription() const;

    virtual bool hasSpatialMissionInfluence() const;

public:
    ActionMotorTest();
    ActionMotorTest(const int &vehicleID);

public:
    void setThrottleSource(const ThrottleChannel &throttleSource)
    {
        this->throttleSource = throttleSource;
    }

    ThrottleChannel getThrottleSource() const
    {
        return throttleSource;
    }

    void setMotorPower(const uint16_t &power)
    {
        this->power = power;
    }

    uint16_t getMotorPower() const
    {
        return this->power;
    }

    void setMotorNumber(const uint16_t &motorNumber)
    {
        this->motorNumber = motorNumber;
    }

    uint16_t getMotorNumber() const
    {
        return motorNumber;
    }


    void setMotorDuration(const uint16_t &duration)
    {
        this->duration = duration;
    }

    uint16_t getMotorDuration() const
    {
        return this->duration;
    }
public:
    void operator = (const ActionMotorTest &rhs)
    {
        AbstractMissionItem::operator =(rhs);
        this->throttleSource = rhs.throttleSource;
        this->power = rhs.power;
        this->motorNumber = rhs.motorNumber;
        this->duration = rhs.duration;
    }

    bool operator == (const ActionMotorTest &rhs) {
        if(!AbstractMissionItem::operator ==(rhs))
        {
            return false;
        }
        if(this->throttleSource != rhs.throttleSource){
            return false;
        }
        if(this->power != rhs.power){
            return false;
        }
        if(this->motorNumber != rhs.motorNumber){
            return false;
        }
        if(this->duration != rhs.duration){
            return false;
        }
        return true;
    }

    bool operator != (const ActionMotorTest &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Action Motor Test(SystemID: "<<m_VehicleID<<", Motor Number: "<<motorNumber<<", Power: "<<power<<", Duration: "<<duration<<")";
        return out;
    }

private:
    ThrottleChannel throttleSource;
    uint16_t power;
    uint16_t motorNumber;
    uint16_t duration;
};

} //end of namespace MissionItem
#endif // ACTION_MOTOR_TEST_H
