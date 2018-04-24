#include "state_takeoff_transitioning.h"

namespace ardupilot{
namespace state{

State_TakeoffTransitioning::State_TakeoffTransitioning():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF_CLIMBING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING;
    desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING;
}

AbstractStateArdupilot* State_TakeoffTransitioning::getClone() const
{
    return (new State_TakeoffTransitioning(*this));
}

void State_TakeoffTransitioning::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_TakeoffTransitioning(*this);
}

hsm::Transition State_TakeoffTransitioning::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_TakeoffTransitioning::handleCommand(const AbstractCommandItem* command)
{

}

void State_TakeoffTransitioning::Update()
{

}

void State_TakeoffTransitioning::OnEnter()
{

}

void State_TakeoffTransitioning::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

