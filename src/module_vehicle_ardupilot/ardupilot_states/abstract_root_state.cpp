#include "abstract_root_state.h"

namespace ardupilot {
namespace state {
AbstractRootState::AbstractRootState(const int &timeout, const int &attempts):
    AbstractStateArdupilot(timeout,attempts)
{

}

AbstractRootState::AbstractRootState(const AbstractRootState &copy):
    AbstractStateArdupilot(copy)
{

}

bool AbstractRootState::handleCommand(const AbstractCommandItem *command)
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

    case COMMANDITEM::CI_NAV_HOME:
    {
        const CommandItem::SpatialHome* cmd = command->as<CommandItem::SpatialHome>();
        auto controllerSystemHome = new MAVLINKVehicleControllers::Command_SetHomeInt(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
        controllerSystemHome->setLambda_Finished([this,controllerSystemHome,cmd](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            {

            }
            controllerSystemHome->Shutdown();
        });

        controllerSystemHome->setLambda_Shutdown([this,controllerSystemHome]()
        {
            currentControllerMutex.lock();
            currentControllers.erase("setHomeController");
            delete controllerSystemHome;
            currentControllerMutex.unlock();
        });

        MaceCore::ModuleCharacteristic target;
        target.ID = Owner().getMAVLINKID();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        MaceCore::ModuleCharacteristic sender;
        sender.ID = 255;
        sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        controllerSystemHome->Send(*cmd,sender,target);
        currentControllers.insert({"setHomeController",controllerSystemHome});
        break;
    }
    default:
        break;
    }
}

} //end of namespace state
} //end of namespace ardupilot
