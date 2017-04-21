#include "ardupilot_takeoff_controller.h"

Ardupilot_TakeoffController::Ardupilot_TakeoffController(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan) :
    Ardupilot_GeneralController(vehicleData, commsMarshaler, linkName, linkChan),
    currentStateLogic(DISARMED)
{
    controllerType = CONTROLLER_TAKEOFF;
    vehicleMissionState = ArdupilotMissionState(2,10,10);
    std::cout << "Constructor on takeoff controller" << std::endl;
}

void Ardupilot_TakeoffController::initializeTakeoffSequence(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &takeoff)
{
    missionItem_Takeoff = takeoff;
    currentVehicleMode = vehicleDataObject->data->getArdupilotFlightMode();
    bool vehicleArmed = currentVehicleMode.getVehicleArmed();

    if((vehicleArmed) && (currentVehicleMode.getFlightModeString() == "GUIDED"))
    {
        currentStateLogic = ARMED_RIGHT_MODE;
        mavlink_message_t msg = vehicleDataObject->generateTakeoffMessage(takeoff,m_LinkChan);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        //the vehicle is already armed and we can send the initial takeoff command

    }else if(vehicleArmed){
        //the vehicle is armed and we should switch to guided
        currentStateLogic = ARMED_WRONG_MODE;
        int requiredMode = currentVehicleMode.getFlightModeFromString("GUIDED");
        int vehicleID = vehicleDataObject->getSystemID();
        mavlink_message_t msg = vehicleDataObject->generateChangeMode(vehicleID,m_LinkChan,requiredMode);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
    }else
    {
        currentStateLogic = DISARMED;
        MissionItem::ActionArm itemArm(vehicleDataObject->getVehicleID(),true);
        //we are in a mode that we can request the aircraft to arm
        mavlink_message_t msg = vehicleDataObject->generateArmMessage(itemArm,m_LinkChan);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
    }
}

void Ardupilot_TakeoffController::updateCommandACK(const mavlink_command_ack_t &cmdACK)
{
    if((cmdACK.command == 22) && (cmdACK.result == MAV_RESULT_ACCEPTED))
    {
        currentStateLogic = ALTITUDE_TRANSITION;
        //The takeoff sequence is being performed
    }
}

double Ardupilot_TakeoffController::distanceToTarget(){
    switch(currentStateLogic)
    {
    case(ALTITUDE_TRANSITION):
    {
        double distance  = fabs(currentPosition.deltaAltitude(missionItem_Takeoff.position));
        return distance;
        break;
    }
    case(HORIZONTAL_TRANSITION):
    {
        break;
    }
    default:
        break;
    }
}

void Ardupilot_TakeoffController::generateControl(const Data::MissionState &currentState)
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
        //we have reached the end of the current mission
        //KEN TODO: We need to figure out what appropriate action to take here
        std::cout<<"I have acheived the takeoff state"<<std::endl;
        mToExit = true;
        break;
    }
    }
}


void Ardupilot_TakeoffController::run()
{
    while(true)
    {
        if(mToExit == true) {
            break;
        }

        switch(currentStateLogic)
        {
        case DISARMED:
        {
            if(modeUpdated)
            {
                if(currentVehicleMode.getVehicleArmed())
                {
                    if(currentVehicleMode.getFlightModeString() == "GUIDED")
                    {
                        currentStateLogic = ARMED_RIGHT_MODE;
                        mavlink_message_t msg = vehicleDataObject->generateTakeoffMessage(missionItem_Takeoff,m_LinkChan);
                        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
                    }else{
                        currentStateLogic = ARMED_WRONG_MODE;
                        int requiredMode = currentVehicleMode.getFlightModeFromString("GUIDED");
                        int vehicleID = vehicleDataObject->getVehicleID();
                        mavlink_message_t msg = vehicleDataObject->generateChangeMode(vehicleID,m_LinkChan,requiredMode);
                        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
                    }

                }
                modeUpdated = false;
            }
            break;
        }
        case ARMED_WRONG_MODE:
        {
            if(modeUpdated)
            {
                if(currentVehicleMode.getVehicleArmed() == false)
                {
                    //for some reason we have taken a step backwards
                    //the vehicle is no longer armed and therefore
                    //we dont want to proceed
                    mToExit = true;
                }
                if(currentVehicleMode.getFlightModeString() == "GUIDED")
                {
                    //we have made a good progression if we are in this phase
                    currentStateLogic = ARMED_RIGHT_MODE;
                    mavlink_message_t msg = vehicleDataObject->generateTakeoffMessage(missionItem_Takeoff,m_LinkChan);
                    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
                }
                modeUpdated = false;
            }
            break;
        }
        case ALTITUDE_TRANSITION:
        {
            if(modeUpdated)
            {
                if((!currentVehicleMode.getVehicleArmed()) || (currentVehicleMode.getFlightModeString() != "GUIDED"))
                {
                    //for some reason we have taken a step backwards
                    //the vehicle is no longer armed and therefore
                    //we dont want to proceed
                    mToExit = true;
                }
            }
            controlSequence();
            break;
        }
        case HORIZONTAL_TRANSITION:
        {
            if(modeUpdated)
            {
                if((!currentVehicleMode.getVehicleArmed()) || (currentVehicleMode.getFlightModeString() != "GUIDED"))
                {
                    //for some reason we have taken a step backwards
                    //the vehicle is no longer armed and therefore
                    //we dont want to proceed
                    mToExit = true;
                }
            }
            controlSequence();
            break;
        }

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void Ardupilot_TakeoffController::controlSequence()
{
    if(positionUpdated)
    {
        //let us see how close we are to our target
        double distance = distanceToTarget();
        std::cout<<"The distance to the target is: "<<distance<<std::endl;
        Data::MissionState currentState = vehicleMissionState.updateMissionState(distance);
        generateControl(currentState);
        positionUpdated = false;

    }
    if(attitudeUpdated)
    {
        attitudeUpdated = false;
    }
}