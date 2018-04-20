#include "state_grounded.h"

namespace ardupilot{
namespace state{

State_Grounded::State_Grounded():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_GROUNDED;
    this->desiredState = ArdupilotFlightState::STATE_GROUNDED;
}

AbstractStateArdupilot* State_Grounded::getClone() const
{
    return (new State_Grounded(*this));
}

void State_Grounded::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Grounded(*this);
}

hsm::Transition State_Grounded::GetTransition()
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
            return hsm::InnerEntryTransition<State_GroundedIdle>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_ARMING:
        {
            return hsm::InnerEntryTransition<State_GroundedArming>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_ARMED:
        {
            return hsm::InnerEntryTransition<State_GroundedArmed>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_DISARMING:
        {
            return hsm::InnerEntryTransition<State_GroundedDisarming>();
            break;
        }
        case ArdupilotFlightState::STATE_TAKEOFF:
        case ArdupilotFlightState::STATE_TAKEOFF_CLIMBING:
        case ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING:
        {
            return hsm::SiblingTransition<State_Takeoff>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

void State_Grounded::handleCommand(const AbstractCommandItem* command)
{
    COMMANDITEM commandType = command->getCommandType();
    switch (commandType) {
    default:
        break;
    }
}

void State_Grounded::Update()
{
    //this update should continue to check if the vehicle is not armed and as such remain in this state
}

void State_Grounded::OnEnter()
{
    if(Owner().state->vehicleArm.get().getSystemArm())
    {
        this->desiredState = ArdupilotFlightState::STATE_GROUNDED_ARMED;
    }
    else
    {
        this->desiredState = ArdupilotFlightState::STATE_GROUNDED_IDLE;
    }
}

void State_Grounded::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded_armed.h"
#include "ardupilot_states/state_grounded_arming.h"
#include "ardupilot_states/state_grounded_disarming.h"
#include "ardupilot_states/state_grounded_idle.h"

#include "ardupilot_states/state_takeoff.h"
