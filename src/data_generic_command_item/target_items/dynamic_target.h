#ifndef DYNAMIC_TARGET_H
#define DYNAMIC_TARGET_H

#include <iostream>
#include <stdint.h>
#include <memory>
#include <list>
#include <map>

#include "base/pose/cartesian_position_3D.h"
#include "base/pose/cartesian_velocity_3D.h"

using namespace mace::pose;

namespace TargetItem {

class DynamicTarget{
public:
    DynamicTarget() = default;

    ~DynamicTarget() = default;

    void setPosition(const mace::pose::CartesianPosition_3D &pos)
    {
        this->position = pos;
    }
    void setVelocity(const mace::pose::CartesianVelocity_3D &vel)
    {
        this->velocity = vel;
    }
    void setYaw(const double &yaw, const double &yawRate)
    {
        this->yaw = yaw;
        this->yawRate = yawRate;
    }

    mace::pose::CartesianPosition_3D getPosition() const
    {
        return this->position;
    }

    mace::pose::CartesianVelocity_3D getVelocity() const
    {
        return this->velocity;
    }

    double getYaw() const
    {
        return this->yaw;
    }

    double getYawRate() const
    {
        return this->yawRate;
    }
public:
    DynamicTarget& operator = (const DynamicTarget &rhs)
    {
        this->position = rhs.position;
        this->velocity = rhs.velocity;
        this->yaw = rhs.yaw;
        this->yawRate = rhs.yawRate;
        return *this;
    }

    bool operator == (const DynamicTarget &rhs) const{
        if(this->position != rhs.position){
            return false;
        }
        if(this->velocity != rhs.velocity){
            return false;
        }
        if(this->yaw != rhs.yaw){
            return false;
        }
        if(this->yawRate != rhs.yawRate){
            return false;
        }
        return true;
    }

    bool operator != (const DynamicTarget &rhs) const{
        return !(*this == rhs);
    }

private:
    mace::pose::CartesianPosition_3D  position;
    mace::pose::CartesianVelocity_3D  velocity;
    double yaw = 0.0;
    double yawRate = 0.0;
};

} //end of namespace TargetItem

#endif // DYNAMIC_TARGET_H
