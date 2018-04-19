#include "state_grounded_disarming.h"

namespace ardupilot{
namespace state{

State_GroundedDisarming::State_GroundedDisarming():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_DISARMING"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_GROUNDED_DISARMING;
    this->desiredState = ArdupilotFlightState::STATE_GROUNDED_DISARMING;
}

AbstractStateArdupilot* State_GroundedDisarming::getClone() const
{
    return (new State_GroundedDisarming(*this));
}

void State_GroundedDisarming::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_GroundedDisarming(*this);
}

hsm::Transition State_GroundedDisarming::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentState != desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredState) {
        case ArdupilotFlightState::STATE_GROUNDED_IDLE:
        {
            return hsm::SiblingTransition<State_GroundedIdle>();
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_GroundedDisarming."<<std::endl;
            break;
        }
    }
    return rtn;
}

void State_GroundedDisarming::handleCommand(const AbstractCommandItem* command)
{

}

void State_GroundedDisarming::Update()
{
    if(Owner().state->vehicleArm.get().getSystemArm() == false)
    {
        this->desiredState = ArdupilotFlightState::STATE_GROUNDED_IDLE;
    }
}

void State_GroundedDisarming::OnEnter()
{

}

void State_GroundedDisarming::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded_idle.h"
#include "ardupilot_states/state_grounded_armed.h"
