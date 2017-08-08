#ifndef HOME_CONTROLLER_EXTERNAL_LINK_H
#define HOME_CONTROLLER_EXTERNAL_LINK_H

#include <iostream>
#include <QDate>
#include "spdlog/spdlog.h"

#include "mace.h"

#include "data/controller_comms_state.h"
#include "data/threadmanager.h"
#include "data/timer.h"

#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_interface_MACE/generic/helper_previous_transmission_mace.h"
#include "data_interface_MACE/COMMS_to_MACE/helper_mission_comms_to_mace.h"
#include "data_interface_MACE/MACE_to_COMMS/helper_mission_mace_to_comms.h"

using namespace DataInterface_MACE;

namespace ExternalLink {

class HomeController_Interface
{
public:
    virtual void cbiHomeController_TransmitHomeReq(const mace_mission_request_home_t &request) = 0;
    virtual void cbiHomeController_ReceviedHome(const CommandItem::SpatialHome &home) = 0;

    virtual void cbiHomeController_TransmitHomeSet(const mace_set_home_position_t &home) = 0;
    virtual void cbiHomeController_ReceivedHomeSetACK(const mace_home_position_ack_t &ack) = 0;
};

class HomeController_ExternalLink : public Thread
{
public:
    HomeController_ExternalLink(HomeController_Interface *cb);

    ~HomeController_ExternalLink() {
        std::cout << "Destructor on the external link home controller" << std::endl;
        mToExit = true;
    }

    void updateLogging(const bool &toLog, const std::string &name);

    void connectCallback(HomeController_Interface *cb)
    {
        m_CB = cb;
    }

    void updateIDS(const int &targetSystem, const int &originSystem);

    void run();

    void requestHome(const int &targetID);
    void setHome(const CommandItem::SpatialHome &home);
    void receivedMissionHome(const mace_home_position_t &systemHome);
    void receivedHomePositionACK(const mace_home_position_ack_t &homeACK);

    Data::ControllerCommsState getCommsState() const
    {
        return this->currentCommsState;
    }

private:
    void clearPreviousTransmit();

private:
    int targetID;
    int transmittingID;

    Timer mTimer;
    int currentRetry;
    int maxRetries;
    int responseTimeout;

private:
    std::shared_ptr<spdlog::logger> mLog;
    HomeController_Interface *m_CB;

    PreviousTransmissionBase<commsItemEnum> *prevTransmit;
    Data::ControllerCommsState currentCommsState;

    CommandItem::SpatialHome missionHome;

protected:
    std::list<std::function<void()>> m_LambdasToRun;

    void clearPendingTasks()
    {
        m_LambdasToRun.clear();
    }

    void RunPendingTasks() {
        while(m_LambdasToRun.size() > 0) {
            auto lambda = m_LambdasToRun.front();
            m_LambdasToRun.pop_front();
            lambda();
        }
    }

};


} //end of namespace DataInterface_MAVLINK

#endif // HOME_CONTROLLER_EXTERNAL_LINK_H
