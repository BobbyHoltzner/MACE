#include "state_takeoff_transitioning.h"

namespace ardupilot{
namespace state{

State_TakeoffTransitioning::State_TakeoffTransitioning():
    AbstractStateArdupilot()
{
    guidedProgress = ArdupilotTargetProgess(2,10,10);
    std::cout<<"We are in the constructor of STATE_TAKEOFF_CLIMBING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING;
    desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING;
}

AbstractStateArdupilot* State_TakeoffTransitioning::getClone() const
{
    return (new State_TakeoffTransitioning(*this));
}

void State_TakeoffTransitioning::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_TakeoffTransitioning(*this);
}

hsm::Transition State_TakeoffTransitioning::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_TakeoffTransitioning::handleCommand(const AbstractCommandItem* command)
{
    clearCommand();
    switch (command->getCommandType()) {
    case COMMANDITEM::CI_NAV_TAKEOFF:
    {
        const CommandItem::SpatialTakeoff* cmd = command->getClone()->as<CommandItem::SpatialTakeoff>();
        if(cmd->getPosition().getPosZFlag())
        {
            Owner().state->vehicleGlobalPosition.AddNotifier(this,[this,cmd]
            {
                if(cmd->getPosition().getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
                {
                    StateGlobalPosition cmdPos(cmd->getPosition().getX(),cmd->getPosition().getY(),cmd->getPosition().getZ());
                    StateGlobalPosition currentPosition = Owner().state->vehicleGlobalPosition.get();
                    double distance = fabs(currentPosition.distanceBetween3D(cmdPos));
                    Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
                    std::cout<<"The distance to the target is approximately: "<<distance<<std::endl;
                    if(guidedState == Data::ControllerState::ACHIEVED)
                    {
                        desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_COMPLETE;
                    }

                }
            });

            auto commandClimb = new MAVLINKVehicleControllers::CommandTakeoff(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
            commandClimb->setLambda_Finished([this,commandClimb](const bool completed, const uint8_t finishCode){
                if(completed && (finishCode != MAV_RESULT_ACCEPTED))
                    commandClimb->Shutdown();
            });

            commandClimb->setLambda_Shutdown([this,commandClimb]()
            {
                currentControllerMutex.lock();
                currentControllers.erase("commandClimb");
                delete commandClimb;
                currentControllerMutex.unlock();
            });

            MaceCore::ModuleCharacteristic target;
            target.ID = cmd->getTargetSystem();
            target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
            MaceCore::ModuleCharacteristic sender;
            sender.ID = 255;
            sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
            commandClimb->Send(*cmd,sender,target);
            currentControllers.insert({"commandTakeoffTransition",commandClimb});
        }
        break;
    }
    default:
        break;
    }
}

void State_TakeoffTransitioning::Update()
{

}

void State_TakeoffTransitioning::OnEnter()
{

}

void State_TakeoffTransitioning::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

