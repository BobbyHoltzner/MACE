#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

AbstractStateArdupilot::AbstractStateArdupilot(const int &timeout, const int &attempts):
    currentCommand(nullptr)
{
    controllerQueue = new Controllers::MessageModuleTransmissionQueue<mavlink_message_t>(timeout, attempts);
}

AbstractStateArdupilot::AbstractStateArdupilot(const AbstractStateArdupilot &copy)
{
    this->currentCommand = copy.currentCommand;
    currentStateEnum = copy.currentStateEnum;
    desiredStateEnum = copy.desiredStateEnum;
}

void AbstractStateArdupilot::OnExit()
{

}


//ArdupilotFlightState AbstractStateArdupilot::getCurrentState() const
//{
//    return currentStateEnum;
//}

//ArdupilotFlightState AbstractStateArdupilot::getDesiredState() const
//{
//    return desiredStateEnum;
//}

void AbstractStateArdupilot::clearCommand()
{
    if(this->currentCommand != nullptr)
    {
        delete currentCommand;
        currentCommand = nullptr;
    }
}

void AbstractStateArdupilot::destroyCurrentControllers()
{
    std::unordered_map<std::string, Controllers::IController<mavlink_message_t>*>::iterator it;
    for(it=currentControllers.begin(); it!=currentControllers.end(); ++it)
    {
        delete it->second;
    }
}
void AbstractStateArdupilot::setCurrentCommand(const AbstractCommandItem *command)
{
    this->currentCommand = command->getClone();
}

bool AbstractStateArdupilot::handleMAVLINKMessage(const mavlink_message_t &msg)
{
    int systemID = msg.sysid;

    MaceCore::ModuleCharacteristic sender;
    sender.ID = systemID;
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    bool consumed = false;
    std::unordered_map<std::string, Controllers::IController<mavlink_message_t>*>::iterator it;
    currentControllerMutex.lock();
    for(it=currentControllers.begin(); it!=currentControllers.end(); ++it)
    {
        Controllers::IController<mavlink_message_t>* obj = it->second;
        consumed = obj->ReceiveMessage(&msg, sender);
    }
    currentControllerMutex.unlock();
    if(!consumed)
    {
        ardupilot::state::AbstractStateArdupilot* childState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
        if(childState != nullptr)
            consumed = childState->handleMAVLINKMessage(msg);
    }
    return consumed;
}

} //end of namespace state
} //end of namespace ardupilot
