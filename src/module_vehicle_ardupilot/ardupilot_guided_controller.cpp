#include "ardupilot_guided_controller.h"


Ardupilot_GuidedController::Ardupilot_GuidedController(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan) :
    vehicleDataObject(vehicleData),
    m_LinkMarshaler(commsMarshaler),m_LinkName(linkName),m_LinkChan(linkChan),
    mToExit(false),vehicleMode(""),executionState(false),
    currentPosition(DataState::StateGlobalPosition()),currentAttitude(DataState::StateAttitude())
{
    vehicleMissionState = ArdupilotMissionState(3,10,10);

    std::cout << "Constructor on guidance controller" << std::endl;
}

void Ardupilot_GuidedController::updateAttitudeTopic(const DataState::StateAttitude &attitude)
{
    attitudeUpdated = true;
    currentAttitude = attitude;
}

void Ardupilot_GuidedController::updateGlobalPositionTopic(const DataState::StateGlobalPosition &globalPosition)
{
    positionUpdated = true;
    currentPosition = globalPosition;
}

void Ardupilot_GuidedController::updatedMission(const MissionItem::MissionList &updatedMission)
{
    m_CurrentMission = updatedMission;
}

void Ardupilot_GuidedController::updatedHomePostion(const MissionItem::SpatialHome &homePosition)
{
     m_VehicleHome = homePosition;
}

void Ardupilot_GuidedController::initializeMissionSequence()
{
    executionState = true;
//    mavlink_message_t msg;
//    mavlink_mission_item_t mavMission;
//    mavMission.autocontinue = 1;
//    mavMission.command = 0;
//    mavMission.current = 2;
//    mavMission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
//    mavMission.param1 = 0.0;
//    mavMission.param2 = 0.0;
//    mavMission.param3 = 0.0;
//    mavMission.param4 = 0.0;
//    mavMission.seq = 0;
//    mavMission.target_system = 0;
//    mavMission.target_component = 0;
//    mavMission.x = 37.892074;
//    mavMission.y = -76.814722;
//    mavMission.z = 100.0;
//    mavlink_msg_mission_item_encode_chan(mSystemID,mCompID,chan,&msg,&tmpItem);

}

double Ardupilot_GuidedController::distanceToTarget(){
    std::shared_ptr<MissionItem::AbstractMissionItem> currentMissionItem = m_CurrentMission.getActiveMissionItem();
    double distance = 0.0;
    switch(currentMissionItem->getMissionType())
    {
    case(MissionItem::MissionItemType::WAYPOINT):
    {
        if(currentMissionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(currentMissionItem);
            DataState::StateGlobalPosition actualPosition(currentPosition);
            actualPosition.altitude = currentPosition.altitude - m_VehicleHome.position.altitude;
            distance = castItem->position.distanceBetween3D(actualPosition);
        }
    break;
    }
    default:
    {
        std::cout<<"I do not understand this type of mission item. I am moving on."<<std::endl;
    }
    }

    return distance;
}

void Ardupilot_GuidedController::generateControl(const Data::MissionState &currentState)
{
    switch(currentState){
    case Data::MissionState::ROUTING:
    {
        //std::cout<<"I am still routing to the waypoint"<<std::endl;
        break;
    }
    case Data::MissionState::HUNTING:
    {
        std::cout<<"I am hunting for the waypoint"<<std::endl;
        break;
    }
    case Data::MissionState::ACHIEVED:
    {
        if(m_CurrentMission.getActiveIndex() == m_CurrentMission.getQueueSize() - 1)
        {
            //we have reached the end of the current mission
            //KEN TODO: We need to figure out what appropriate action to take here
        }
        else{
            m_CurrentMission.setActiveIndex(m_CurrentMission.getActiveIndex() + 1);
        }
        std::cout<<"I have acheived the waypoint"<<std::endl;
        break;
    }
    }
}


void Ardupilot_GuidedController::run()
{
    while(true)
    {
        if(mToExit == true) {
            break;
        }
        if(positionUpdated)
        {
            if(executionState){
                //let us see how close we are to our target
                double distance = distanceToTarget();

                std::cout<<"The distance to the target is: "<<distance<<std::endl;
                Data::MissionState currentState = vehicleMissionState.updateMissionState(distance);
                generateControl(currentState);
            }
            positionUpdated = false;

        }
        if(attitudeUpdated)
        {
            attitudeUpdated = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
