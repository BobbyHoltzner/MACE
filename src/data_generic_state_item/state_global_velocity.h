#ifndef STATE_GLOBAL_VELOCITY_H
#define STATE_GLOBAL_VELOCITY_H

namespace DataState {

class StateGlobalVelocity
{
public:
    StateGlobalVelocity();
public:
    float x;
    float y;
    float z;
    float heading;
};

} //end of namespace DataState

#endif // STATE_GLOBAL_VELOCITY_H
