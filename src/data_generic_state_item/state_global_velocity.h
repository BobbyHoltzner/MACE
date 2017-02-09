#ifndef STATE_GLOBAL_VELOCITY_H
#define STATE_GLOBAL_VELOCITY_H

namespace DataState {

class StateGlobalVelocity
{
public:
    StateGlobalVelocity();
public:
    double x;
    double y;
    double z;
    double heading;
};

} //end of namespace DataState

#endif // STATE_GLOBAL_VELOCITY_H
