#include "state_landing.h"

namespace ardupilot{
namespace state{

State_Landing::State_Landing():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_LANDING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_LANDING;
    desiredStateEnum = ArdupilotFlightState::STATE_LANDING;
}

AbstractStateArdupilot* State_Landing::getClone() const
{
    return (new State_Landing(*this));
}

void State_Landing::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Landing(*this);
}

hsm::Transition State_Landing::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        if(IsInInnerState<State_LandingComplete>())
        {
            rtn = hsm::SiblingTransition<State_Grounded>();
        }
        else
        {
            //this means we want to chage the state of the vehicle for some reason
            //this could be caused by a command, action sensed by the vehicle, or
            //for various other peripheral reasons
            switch (desiredStateEnum) {
            case ArdupilotFlightState::STATE_LANDING_DESCENDING:
            {
                rtn = hsm::InnerEntryTransition<State_LandingDescent>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_LANDING_TRANSITIONING:
            {
                rtn = hsm::InnerEntryTransition<State_LandingTransitioning>(currentCommand);
                break;
            }
            default:
                std::cout<<"I dont know how we eneded up in this transition state from STATE_TAKEOFF."<<std::endl;
                break;
            }
        }
    }
}

bool State_Landing::handleCommand(const AbstractCommandItem* command)
{

}

void State_Landing::Update()
{

}

void State_Landing::OnEnter()
{

}

void State_Landing::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_landing_descent.h"
#include "ardupilot_states/state_landing_transitioning.h"
#include "ardupilot_states/state_landing_complete.h"
#include "ardupilot_states/state_grounded.h"
