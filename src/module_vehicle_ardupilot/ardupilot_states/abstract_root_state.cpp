#include "abstract_root_state.h"

namespace ardupilot {
namespace state {
AbstractRootState::AbstractRootState():
    AbstractStateArdupilot()
{

}

AbstractRootState::AbstractRootState(const AbstractRootState &copy):
    AbstractStateArdupilot(copy)
{

}

bool AbstractRootState::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{

    switch (command->getCommandType()) {
    case COMMANDITEM::CI_ACT_CHANGEMODE:
    {
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerSystemMode = new MAVLINKVehicleControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){

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
        controllerSystemMode->Send(commandMode,sender,target);
        collection->Insert("modeController",controllerSystemMode);
        break;
    }

    case COMMANDITEM::CI_NAV_HOME:
    {
        const CommandItem::SpatialHome* cmd = command->as<CommandItem::SpatialHome>();
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerSystemHome = new MAVLINKVehicleControllers::Command_SetHomeInt(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemHome->AddLambda_Finished(this, [this,controllerSystemHome,cmd](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            {

            }
            controllerSystemHome->Shutdown();
        });

        controllerSystemHome->setLambda_Shutdown([this, collection]()
        {
            auto ptr = collection->Remove("setHomeController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        controllerSystemHome->Send(*cmd,sender,target);
        collection->Insert("setHomeController",controllerSystemHome);
        break;
    }
    default:
        break;
    }
}

} //end of namespace state
} //end of namespace ardupilot
