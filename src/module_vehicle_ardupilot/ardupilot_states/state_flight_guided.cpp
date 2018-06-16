#include "state_flight_guided.h"

namespace ardupilot{
namespace state{

State_FlightGuided::State_FlightGuided():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
}

AbstractStateArdupilot* State_FlightGuided::getClone() const
{
    return (new State_FlightGuided(*this));
}

void State_FlightGuided::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided(*this);
}

hsm::Transition State_FlightGuided::GetTransition()
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

bool State_FlightGuided::handleCommand(const AbstractCommandItem* command)
{

}

void State_FlightGuided::Update()
{

}

void State_FlightGuided::OnEnter()
{
    Controllers::ControllerCollection<mavlink_message_t> *collection = Owner().ControllersCollection();

    auto controllerGuided = new MAVLINKVehicleControllers::ControllerGuidedTargetItem<MAVLINKVehicleControllers::TargetControllerStruct>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerGuided->AddLambda_Finished(this, [this, controllerGuided](const bool completed, const uint8_t finishCode){
        controllerGuided->Shutdown();

//        if(!completed || (finishCode != MAV_RESULT_ACCEPTED))
//        {
//            std::cout<<"The ardupilot rejected this command"<<std::endl;
//        }
    });

    controllerGuided->setLambda_Shutdown([this, collection]()
    {
        auto ptr = collection->Remove("guidedController");
        delete ptr;
    });

    MaceCore::ModuleCharacteristic target;
    target.ID = Owner().getMAVLINKID();
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
    MaceCore::ModuleCharacteristic sender;
    sender.ID = 255;
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    TargetItem::DynamicTargetList::DynamicTarget dynamicTarget;
    dynamicTarget.position.setXPosition(-10);
    dynamicTarget.position.setYPosition(-10);
    dynamicTarget.position.setZPosition(-20);


    MAVLINKVehicleControllers::TargetControllerStruct action;
    action.targetID = target.ID;
    action.target = dynamicTarget;


    controllerGuided->Send(action, sender, target);
    collection->Insert("guidedController", controllerGuided);
}

void State_FlightGuided::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

