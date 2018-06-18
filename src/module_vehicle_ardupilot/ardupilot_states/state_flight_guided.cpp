#include "state_flight_guided.h"

namespace ardupilot{
namespace state{

State_FlightGuided::State_FlightGuided():
    AbstractStateArdupilot(), guidedTimeout(nullptr), currentQueue(nullptr)
{
    guidedTimeout = new GuidedTimeoutController(1000);
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;

    guidedTimeout->connectTargetCallback(State_FlightGuided::staticCallbackFunction_VehicleTarget,this);
}

void State_FlightGuided::OnExit()
{
    guidedTimeout->stop();
    delete guidedTimeout;

    delete currentQueue;
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
//Once we get the command that we can go, we need to announce the current mission item
//    MissionItem::MissionKey key = Owner().mission->missionItemCurrent.get().getMissionKey();
//    unsigned int index = Owner().mission->missionItemCurrent.get().getMissionCurrentIndex();

//    MissionItem::MissionItemCurrent currentMissionItem(key,index);
//    Owner().m_CB->cbi_VehicleMissionItemCurrent(currentMissionItem);
}

void State_FlightGuided::Update()
{

}

void State_FlightGuided::OnEnter()
{
    TargetItem::DynamicMissionQueue availableQueue;
    availableQueue.describingMissionItem = 1;
    MissionItem::MissionKey testKey(1,1,1,MissionItem::MISSIONTYPE::GUIDED);
    availableQueue.missionKey = testKey;

    TargetItem::DynamicTarget target;
    target.position.setXPosition(1000);
    target.position.setYPosition(1000);
    target.position.setZPosition(-100);

    availableQueue.m_TargetList.appendDynamicTarget(target,TargetItem::DynamicTargetStorage::INCOMPLETE);

    //    Owner().mission->currentDynamicQueue.set(availableQueue);

//    Owner().mission->currentDynamicQueue.AddNotifier(this,[this]{
//        //std::lock_guard<std::mutex> guard(MUTEXTargetQueue);
//        currentQueue = new TargetItem::DynamicMissionQueue(Owner().mission->currentDynamicQueue.get());
//    });

    currentQueue = new TargetItem::DynamicMissionQueue(availableQueue);// new TargetItem::DynamicMissionQueue(Owner().mission->currentDynamicQueue.get());
    Owner().state->vehicleLocalPosition.AddNotifier(this,[this]
    {
        //std::lock_guard<std::mutex> guard(MUTEXTargetQueue);
        DataState::StateLocalPosition CP = Owner().state->vehicleLocalPosition.get();
        mace::pose::CartesianPosition_3D currentPosition(CP.getPositionX(),CP.getPositionY(),CP.getPositionZ());
        std::cout<<"The current position here is: "<<CP.getPositionX()<<" "<<CP.getPositionY()<<std::endl;
        unsigned int currentTargetIndex = currentQueue->m_TargetList.getActiveTargetItem();
        const TargetItem::DynamicTargetList::DynamicTarget* target = currentQueue->m_TargetList.getTargetPointerAtIndex(currentTargetIndex);
        double distance = currentPosition.distanceBetween3D(target->position);
        Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
        handleGuidedState(currentPosition, currentTargetIndex, guidedState, distance);
    });

}

void State_FlightGuided::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

void State_FlightGuided::initializeNewTargetList()
{

}

void State_FlightGuided::handleGuidedState(const mace::pose::CartesianPosition_3D currentPosition, const unsigned int currentTargetIndex,
                                           const Data::ControllerState &state, const double targetDistance)
{
    if(state == Data::ControllerState::ACHIEVED)
    {

        const TargetItem::DynamicTargetList::DynamicTarget* newTarget = currentQueue->m_TargetList.markCompletionState(currentTargetIndex,TargetItem::DynamicTargetList::DynamicTargetStorage::TargetCompletion::COMPLETE);
        if(newTarget == nullptr)
        {
            //if there are no more points in the queue this mission item is completed
            MissionItem::MissionItemAchieved achievement(currentQueue->missionKey,currentQueue->describingMissionItem);
            std::shared_ptr<MissionTopic::MissionItemReachedTopic> ptrMissionTopic = std::make_shared<MissionTopic::MissionItemReachedTopic>(achievement);
            Owner().getCallbackInterface()->cbi_VehicleMissionData(Owner().getMAVLINKID(),ptrMissionTopic);
        }
        else
        {
            unsigned int currentTargetIndex = currentQueue->m_TargetList.getActiveTargetItem();
            const TargetItem::DynamicTargetList::DynamicTarget* target = currentQueue->m_TargetList.getTargetPointerAtIndex(currentTargetIndex);
            double distance = currentPosition.distanceBetween3D(target->position);
            Data::ControllerState guidedState = guidedProgress.newTargetItem(distance);
            handleGuidedState(currentPosition, currentTargetIndex, guidedState, distance);
        }
        //advance to the next desired dynamic state
    }
    else //we are either hunting or tracking the state
    {
        //these items are going to be in the local coordinate frame relative to the vehicle home
        CommandItem::SpatialHome home = Owner().mission->vehicleHomePosition.get();
        mace::pose::GeodeticPosition_3D homePos(home.getPosition().getX(),home.getPosition().getY(),home.getPosition().getZ());
        GeodeticPosition_3D targetPosition;
        DynamicsAid::LocalPositionToGlobal(homePos,currentPosition,targetPosition);
        Base3DPosition targetPositionCast(targetPosition.getLatitude(),targetPosition.getLongitude(),targetPosition.getLongitude());
        MissionTopic::VehicleTargetTopic currentTarget(Owner().getMAVLINKID(),targetPositionCast, targetDistance);
        Owner().callTargetCallback(currentTarget);
    }
}

} //end of namespace ardupilot
} //end of namespace state

