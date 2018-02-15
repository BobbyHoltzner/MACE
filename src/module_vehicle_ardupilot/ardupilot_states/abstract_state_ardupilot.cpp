#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

AbstractStateArdupilot::AbstractStateArdupilot()
{

}

AbstractStateArdupilot::AbstractStateArdupilot(const AbstractStateArdupilot &copy)
{
    this->currentState = copy.currentState;
    this->desiredState = copy.desiredState;
}

ArdupilotFlightState AbstractStateArdupilot::getCurrentState() const
{
    return currentState;
}

ArdupilotFlightState AbstractStateArdupilot::getDesiredState() const
{
    return desiredState;
}

void AbstractStateArdupilot::clearCommand()
{

}


} //end of namespace state
} //end of namespace ardupilot
