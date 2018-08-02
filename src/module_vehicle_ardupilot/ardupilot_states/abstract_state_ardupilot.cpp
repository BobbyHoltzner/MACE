#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

AbstractStateArdupilot::AbstractStateArdupilot() :
    currentCommand(nullptr)

{
}

AbstractStateArdupilot::AbstractStateArdupilot(const AbstractStateArdupilot &copy)
{
    this->currentCommand = copy.currentCommand->getClone();
    currentStateEnum = copy.currentStateEnum;
    desiredStateEnum = copy.desiredStateEnum;
}

void AbstractStateArdupilot::OnExit()
{
    Owner().ControllersCollection()->ForAll([this](Controllers::IController<mavlink_message_t, int>* controller){
        controller->RemoveHost(this);
    });
}

void AbstractStateArdupilot::clearCommand()
{
    if(this->currentCommand != nullptr)
    {
        delete currentCommand;
        currentCommand = nullptr;
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
        Controllers::ControllerCollection<mavlink_message_t, int> *collection = Owner().ControllersCollection();
        auto controllerSystemMode = new MAVLINKVehicleControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->AddLambda_Finished(this, [this, controllerSystemMode](const bool completed, const uint8_t finishCode){

            controllerSystemMode->Shutdown();
        });

        controllerSystemMode->setLambda_Shutdown([this, collection]()
        {
            auto ptr = collection->Remove("modeController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        MAVLINKVehicleControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = Owner().getMAVLINKID();
        commandMode.vehicleMode = Owner().ardupilotMode.getFlightModeFromString(command->as<CommandItem::ActionChangeMode>()->getRequestMode());
        controllerSystemMode->Send(commandMode, sender, target);

        collection->Insert("modeController", controllerSystemMode);

        break;
    }
    default:
        break;
    }
}

bool AbstractStateArdupilot::handleMAVLINKMessage(const mavlink_message_t &msg)
{
    throw std::runtime_error("States should not longer handle mavlink messages");

    /*
    int systemID = msg.sysid;

    MaceCore::ModuleCharacteristic sender;
    sender.ModuleID = systemID;
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    bool consumed = false;
    std::unordered_map<std::string, Controllers::IController<mavlink_message_t>*>::iterator it;

    m_ControllerFactory->controllerMutex.lock();
    for(it=m_ControllerFactory->controllers.begin(); it!=m_ControllerFactory->controllers.end(); ++it)
    {
        Controllers::IController<mavlink_message_t>* obj = it->second;
        consumed = obj->ReceiveMessage(&msg, sender);
    }
    m_ControllerFactory->controllerMutex.unlock();


    if(!consumed)
    {
        State* innerState = GetImmediateInnerState();
        if(innerState == this)
        {
            printf("!!!!!! WARNING: Immediate Inner State is equal to the outer state. This is a non-op that will result in infinte recursion. Ignoring but it probably points to a larger bug\n");
            return consumed;
        }

        if(innerState != nullptr)
        {
            ardupilot::state::AbstractStateArdupilot* castChild = static_cast<ardupilot::state::AbstractStateArdupilot*>(innerState);
            consumed = castChild->handleMAVLINKMessage(msg);
        }

    }
    return consumed;
    */
    return false;
}

} //end of namespace state
} //end of namespace ardupilot
