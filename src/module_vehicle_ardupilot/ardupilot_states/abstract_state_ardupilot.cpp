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
void AbstractStateArdupilot::setCurrentCommand(const CommandItem::AbstractCommandItem *command)
{
    this->currentCommand = command->getClone();
}

bool AbstractStateArdupilot::handleCommand(const CommandItem::AbstractCommandItem *command)
{
    switch (command->getCommandType()) {
    case COMMANDITEM::CI_ACT_CHANGEMODE:
    {
        auto controllerSystemMode = new MAVLINKVehicleControllers::ControllerSystemMode(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->setLambda_Finished([this,controllerSystemMode](const bool completed, const uint8_t finishCode){

            controllerSystemMode->Shutdown();
        });

        controllerSystemMode->setLambda_Shutdown([this,controllerSystemMode]()
        {
            currentControllerMutex.lock();
            currentControllers.erase("modeController");
            delete controllerSystemMode;
            currentControllerMutex.unlock();
        });

        MaceCore::ModuleCharacteristic target;
        target.ID = Owner().getMAVLINKID();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        MaceCore::ModuleCharacteristic sender;
        sender.ID = 255;
        sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        MAVLINKVehicleControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = target.ID;
        commandMode.vehicleMode = Owner().ardupilotMode.getFlightModeFromString(command->as<CommandItem::ActionChangeMode>()->getRequestMode());
        controllerSystemMode->Send(commandMode,sender,target);
        currentControllers.insert({"modeController",controllerSystemMode});
        break;
    }
    default:
        break;
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
