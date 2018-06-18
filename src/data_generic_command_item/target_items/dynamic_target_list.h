#ifndef DYNAMIC_TARGET_LIST_H
#define DYNAMIC_TARGET_LIST_H
#include <iostream>
#include <stdint.h>
#include <memory>
#include <list>
#include <map>

#include "../mission_items/mission_list.h"

#include "base/pose/cartesian_position_3D.h"
#include "base/pose/cartesian_velocity_3D.h"

namespace TargetItem {

class DynamicTargetList
{
public:
    struct DynamicTarget{
        mace::pose::CartesianPosition_3D  position;
        mace::pose::CartesianVelocity_3D  velocity;
        double yaw = 0.0;
        double yawRate = 0.0;

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
    };

public:
    enum TargetCompletion{
        COMPLETE,
        ACTIVE,
        INCOMPLETE
    };

public:
    struct DynamicTargetStorage
    {
        DynamicTargetStorage(const DynamicTarget &target, const TargetCompletion &state)
        {
            this->target = target;
            this->state = state;
        }

        DynamicTargetStorage& operator = (const DynamicTargetStorage &rhs)
        {
            this->target = rhs.target;
            this->state = rhs.state;
            return *this;
        }

        bool operator == (const DynamicTargetStorage &rhs) const{
            if(this->target != rhs.target){
                return false;
            }
            if(this->state != rhs.state){
                return false;
            }
            return true;
        }

        bool operator != (const DynamicTargetStorage &rhs) const{
            return !(*this == rhs);
        }


        DynamicTarget target;
        TargetCompletion state;
    };

public:
    DynamicTargetList();
    DynamicTargetList(const DynamicTargetList &rhs);

public:
    size_t listSize() const;
    void clearList();

    void appendDynamicTarget(const DynamicTarget &target, const TargetCompletion &state = TargetCompletion::INCOMPLETE);
    void removeTargetAtIndex(const unsigned int &index);

    void replaceTargetAtIndex(const unsigned int &index, const DynamicTarget &target, const TargetCompletion &state = TargetCompletion::INCOMPLETE);
    void spliceTargetListAtIndex(const unsigned int &index, const std::list<DynamicTargetStorage> &list);

    bool isCompleted() const;

    unsigned int getActiveTargetItem() const;


public:
    const DynamicTargetStorage* getTargetStorageAtIndex(const unsigned int &index) const;
    DynamicTarget getTargetAtIndex(const unsigned int &index) const;
    const DynamicTarget* getTargetPointerAtIndex(const unsigned int &index) const;
    const DynamicTarget* getNextIncomplete() const;
    const DynamicTarget* markCompletionState(const unsigned int &index, const TargetCompletion &state);

public:

    DynamicTargetList& operator = (const DynamicTargetList &rhs)
    {
        this->activeTargetItem = rhs.activeTargetItem;
        this->targetList = rhs.targetList;
        return *this;
    }

    bool operator == (const DynamicTargetList &rhs) const{
        if(this->activeTargetItem != rhs.activeTargetItem){
            return false;
        }
        if(this->targetList != rhs.targetList){
            return false;
        }
        return true;
    }

    bool operator != (const DynamicTargetList &rhs) const{
        return !(*this == rhs);
    }

private:

    std::list<DynamicTargetStorage> targetList;
    unsigned int activeTargetItem = 0;

public:
    friend std::ostream& operator<<(std::ostream& os, const DynamicTargetList& t);

};

} //end of namespace MissionItem
#endif // DYNAMIC_TARGET_LIST_H
