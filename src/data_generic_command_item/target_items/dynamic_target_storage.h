#ifndef DYNAMIC_TARGET_STORAGE_H
#define DYNAMIC_TARGET_STORAGE_H

#include "dynamic_target.h"

namespace TargetItem {

class DynamicTargetStorage
{
public:
    enum TargetCompletion{
        COMPLETE,
        ACTIVE,
        INCOMPLETE
    };

    DynamicTargetStorage(const DynamicTarget &target, const TargetCompletion &state)
    {
        this->target = target;
        this->state = state;
    }

public:
    void setDynamicTarget(const DynamicTarget &target)
    {
        this->target = target;
    }

    void setTargetState(const TargetCompletion &state)
    {
        this->state = state;
    }

    const DynamicTarget* getDynamicTarget() const
    {
        return &this->target;
    }

    DynamicTarget getDynamicTargetCopy() const
    {
        return this->target;
    }

    TargetCompletion getTargetState() const
    {
        return this->state;
    }

public:
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

private:
    DynamicTarget target;
    TargetCompletion state;
};

} //end of namespace TargetItem

#endif // DYNAMIC_TARGET_STORAGE_H