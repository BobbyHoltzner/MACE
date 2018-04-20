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

bool AbstractStateArdupilot::handleMAVLINKMessage(const mavlink_message_t &msg)
{
    int systemID = msg.sysid;

    MaceCore::ModuleCharacteristic sender;
    sender.ID = systemID;
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    bool consumed = false;
    std::unordered_map<std::string, Controllers::IController<mavlink_message_t>*>::iterator it;
    for(it=currentControllers.begin(); it!=currentControllers.end(); ++it)
    {
        Controllers::IController<mavlink_message_t>* obj = it->second;
        if(msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK)
            std::cout<<"We definitely saw an ack"<<std::endl;
        consumed = obj->ReceiveMessage(&msg, sender);
    }
    return consumed;
}

} //end of namespace state
} //end of namespace ardupilot
