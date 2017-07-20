#include "ardupilot_guided_controller.h"


Ardupilot_GuidedController::Ardupilot_GuidedController(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan, callbackFunction callback) :
    Ardupilot_GeneralController(vehicleData, commsMarshaler, linkName, linkChan, callback)
{
    controllerType = CONTROLLER_GUIDED;
    vehicleMissionState = ArdupilotMissionState(3,10,10);
    std::cout << "Constructor on guidance controller" << std::endl;
}

void Ardupilot_GuidedController::updatedMission(const MissionItem::MissionList &updatedMission)
{
    //KEN TODO: Do these types of items need to be thread protected since
    //they can be accessed and changed in the controllers indpendent thread
    m_CurrentMission = updatedMission;
}

void Ardupilot_GuidedController::updateCommandACK(const mavlink_command_ack_t &cmdACK)
{
    UNUSED(cmdACK);
}

void Ardupilot_GuidedController::initializeMissionSequence()
{
    executionState = true;
    std::shared_ptr<CommandItem::AbstractCommandItem> missionItem = m_CurrentMission.getActiveMissionItem();
    mavlink_message_t msg;
    vehicleDataObject->generateBasicGuidedMessage(missionItem,m_LinkChan,msg);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

double Ardupilot_GuidedController::distanceToTarget(){
    std::shared_ptr<CommandItem::AbstractCommandItem> currentMissionItem = m_CurrentMission.getActiveMissionItem();
    double distance = 0.0;
    switch(currentMissionItem->getCommandType())
    {
    case(Data::CommandItemType::CI_NAV_WAYPOINT):
    {
        if(currentMissionItem->getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            std::shared_ptr<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>>(currentMissionItem);
            DataState::StateGlobalPosition currentPosition = vehicleDataObject->data->vehicleGlobalPosition.get();
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

void Ardupilot_GuidedController::generateControl(const Data::ControllerState &currentState)
{
    switch(currentState){
    case Data::ControllerState::TRACKING:
    {
        //std::cout<<"I am still routing to the waypoint"<<std::endl;
        break;
    }
    case Data::ControllerState::HUNTING:
    {
        std::cout<<"I am hunting for the waypoint"<<std::endl;
        break;
    }
    case Data::ControllerState::ACHIEVED:
    {
        if(m_CurrentMission.getActiveIndex() == m_CurrentMission.getQueueSize() - 1)
        {
            //we have reached the end of the current mission
            //KEN TODO: We need to figure out what appropriate action to take here
        }
        else{
            m_CurrentMission.setActiveIndex(m_CurrentMission.getActiveIndex() + 1);
            std::shared_ptr<CommandItem::AbstractCommandItem> missionItem = m_CurrentMission.getActiveMissionItem();
            mavlink_message_t msg;
            vehicleDataObject->generateBasicGuidedMessage(missionItem,m_LinkChan,msg);
            m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
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

            if(executionState){
                //let us see how close we are to our target
                double distance = distanceToTarget();

                std::cout<<"The distance to the target is: "<<distance<<std::endl;
                Data::ControllerState currentState = vehicleMissionState.updateMissionState(distance);
                generateControl(currentState);
            }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
