#include "home_controller_external_link.h"

namespace ExternalLink {

HomeController_ExternalLink::HomeController_ExternalLink(HomeController_Interface *cb):
    targetID(0), transmittingID(0),mLog(NULL),
    currentRetry(0), maxRetries(5), responseTimeout(5000),\
    currentCommsState(Data::ControllerCommsState::NEUTRAL),
    m_CB(NULL), prevTransmit(NULL)
{
    connectCallback(cb);
}

void HomeController_ExternalLink::updateLogging(const bool &toLog, const std::string &name)
{
    UNUSED(toLog);
    mLog = spdlog::get(name);
}

void HomeController_ExternalLink::clearPreviousTransmit()
{
    if(prevTransmit)
    {
        delete prevTransmit;
        prevTransmit = NULL;
    }
}

void HomeController_ExternalLink::updateIDS(const int &targetSystem, const int &originSystem)
{
    this->targetID = targetSystem; //this is to whom this mission would be going to
    this->transmittingID = originSystem; //this is to whom this mission is from
}

///////////////////////////////////////////////////////////////////////////////
/// GENERAL TRANSMISSION EVENTS: These events are related to sending a mission
/// to a remote instance of MACE.
///////////////////////////////////////////////////////////////////////////////


void HomeController_ExternalLink::run()
{
    while(true)
    {
        if(mToExit == true) {
            clearPendingTasks();
            clearPreviousTransmit();
            mTimer.stop();
            break;
        }

        this->RunPendingTasks();

        //The current state we can find out how much time has passed.
        //If one of the lambda expressions has fired the clock shoud
        //be reset right at the end, thus making this value small and
        //improbable the next function will fire
        double timeElapsed = mTimer.elapsedMilliseconds();

        if(timeElapsed > responseTimeout)
        {
            commsItemEnum type = prevTransmit->getType();
            currentRetry++;

            switch(currentCommsState)
            {
            case Data::ControllerCommsState::NEUTRAL:
            {
                //This case we should terminate this because there is nothing we should be doing apparently
                clearPreviousTransmit();
                mTimer.stop();
                mToExit = true;
                break;
            }
            case Data::ControllerCommsState::RECEIVING:
            {
                if(type == commsItemEnum::ITEM_RXHOME)
                {
                    //mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    std::cout<<"Making another request for home item"<<std::endl;
                    PreviousTransmission<mace_mission_request_home_t> *tmp = static_cast<PreviousTransmission<mace_mission_request_home_t>*>(prevTransmit);
                    mace_mission_request_home_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiHomeController_TransmitHomeReq(msgTransmit);
                }
                break;
            }
            case Data::ControllerCommsState::TRANSMITTING:
            {
                if(type == commsItemEnum::ITEM_SETHOME)
                {
                    //mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    std::cout<<"Making another request for home item"<<std::endl;
                    PreviousTransmission<mace_set_home_position_t> *tmp = static_cast<PreviousTransmission<mace_set_home_position_t>*>(prevTransmit);
                    mace_set_home_position_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiHomeController_TransmitHomeSet(msgTransmit);
                }
                break;
            }
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}


void HomeController_ExternalLink::requestHome(const int &systemID)
{
    //mLog->info("Mission Controller has seen a request home.");
    std::cout<<"Mission controller is making a request for the home position"<<std::endl;
    currentCommsState = Data::ControllerCommsState::RECEIVING;

    mace_mission_request_home_t request;
    request.target_system = systemID;

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mace_mission_request_home_t>(commsItemEnum::ITEM_RXHOME, request);

    if(m_CB)
        m_CB->cbiHomeController_TransmitHomeReq(request);

    currentRetry = 0;
    this->start();
    mTimer.start();
}

void HomeController_ExternalLink::setHome(const CommandItem::SpatialHome &home)
{
    if(home.position.isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
    {
        mace_set_home_position_t setHome;
        setHome.target_system = home.getTargetSystem();
        setHome.latitude = home.position.getX() * pow(10,7);
        setHome.longitude = home.position.getY()* pow(10,7);
        setHome.altitude = home.position.getZ() * 1000.0;

        clearPendingTasks();
        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mace_set_home_position_t>(commsItemEnum::ITEM_SETHOME, setHome);

        currentRetry = 0;
        this->start();
        mTimer.start();

        m_CB->cbiHomeController_TransmitHomeSet(setHome);
    }
}

void HomeController_ExternalLink::receivedMissionHome(const mace_home_position_t &home)
{
    m_LambdasToRun.push_back([this, home]{
        mTimer.stop();
        currentRetry = 0;
        std::cout<<"Mission controller received the home position"<<std::endl;

        //mLog->info("Mission Controller received system home item item.");

        //This is the home position item associated with the vehicle
        CommandItem::SpatialHome newHome;
        newHome.position.setX(home.latitude / pow(10,7));
        newHome.position.setY(home.longitude / pow(10,7));
        newHome.position.setZ(home.altitude / pow(10,7));
        newHome.setOriginatingSystem(targetID);
        newHome.setTargetSystem(targetID);

        clearPendingTasks();
        mToExit = true;
        currentCommsState = Data::ControllerCommsState::NEUTRAL;

        m_CB->cbiHomeController_ReceviedHome(newHome);
    });
}

void HomeController_ExternalLink::receivedHomePositionACK(const mace_home_position_ack_t &homeACK)
{
    if(!isThreadActive())
    {
        std::cout<<"The thread is not currently active so this must be something else"<<std::endl;
        return;
    }

    m_LambdasToRun.push_back([this, homeACK]{
        std::cout<<"Mission Controller received a mission ack"<<std::endl;
        mToExit = true;
        mTimer.stop();
        currentRetry = 0;
        clearPendingTasks();
        clearPreviousTransmit();
        currentCommsState = Data::ControllerCommsState::NEUTRAL;
        if(m_CB)
            m_CB->cbiHomeController_ReceivedHomeSetACK(homeACK);

    });
}

} //end of namespace DataInterface_MAVLINK

