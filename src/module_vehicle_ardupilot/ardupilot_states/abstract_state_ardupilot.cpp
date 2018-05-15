#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

AbstractStateArdupilot::AbstractStateArdupilot(ControllerFactory *controllerFactory) :
    currentCommand(nullptr),
    m_ControllerFactory(controllerFactory)

{
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

void AbstractStateArdupilot::setCurrentCommand(const CommandItem::AbstractCommandItem *command)
{
    this->currentCommand = command->getClone();
}

bool AbstractStateArdupilot::handleCommand(const CommandItem::AbstractCommandItem *command)
{
    switch (command->getCommandType()) {
    case COMMANDITEM::CI_ACT_CHANGEMODE:
    {
        auto controllerSystemMode = new MAVLINKVehicleControllers::ControllerSystemMode(&Owner(), &m_ControllerFactory->messageQueue, Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->setLambda_Finished([this,controllerSystemMode](const bool completed, const uint8_t finishCode){

            controllerSystemMode->Shutdown();
        });

        controllerSystemMode->setLambda_Shutdown([this,controllerSystemMode]()
        {
            m_ControllerFactory->controllerMutex.lock();
            m_ControllerFactory->controllers.erase("modeController");
            delete controllerSystemMode;
            m_ControllerFactory->controllerMutex.unlock();
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
        m_ControllerFactory->controllers.insert({"modeController",controllerSystemMode});
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
}

} //end of namespace state
} //end of namespace ardupilot
