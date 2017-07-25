#ifndef MISSION_CONTROLLER_EXTERNALLINK_H
#define MISSION_CONTROLLER_EXTERNALLINK_H

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

//typedef void(*CallbackFunctionPtr_MisCount)(void*, mace_message_t &);

using namespace DataInterface_MACE;

namespace ExternalLink {

class MissionController_Interface
{
public:
    virtual void cbiMissionController_TransmitMissionCount(const mace_mission_count_t &count) = 0;
    virtual void cbiMissionController_TransmitMissionItem(const mace_mission_item_t &item) = 0;

    virtual void cbiMissionController_TransmitMissionReqList(const mace_mission_request_list_t &request) = 0;
    virtual void cbiMissionController_TransmitMissionReq(const mace_mission_request_item_t &requestItem) = 0;

    virtual void cbiMissionController_TransmitHomeReq(const mace_mission_request_home_t &request) = 0;

    virtual void cbiMissionController_ReceviedHome(const CommandItem::SpatialHome &home) = 0;
    virtual void cbiMissionController_ReceivedMission(const MissionItem::MissionList &missionList) = 0;
    virtual void cbiMissionController_MissionACK(const mace_mission_ack_t &missionACK) = 0;
};

class MissionController_ExternalLink : public Thread
{
public:
    MissionController_ExternalLink(MissionController_Interface *cb);

    ~MissionController_ExternalLink() {
        std::cout << "Destructor on the mavlink mission controller" << std::endl;
        mToExit = true;
    }

    void connectCallback(MissionController_Interface *cb)
    {
        m_CB = cb;
    }

    void updateIDS(const int &targetID, const int &originatingID);

    void run();

    //These 3 functions are related to transmitting a mission
    void transmitMission(const MissionItem::MissionList &missionQueue);
    void transmitMissionItem(const mace_mission_request_item_t &missionRequest);
    void receivedMissionACK(const mace_mission_ack_t &missionACK);

    void receivedMissionCount(const mace_mission_count_t &mission);
    void recievedMissionItem(const mace_mission_item_t &missionItem);

    //void requestMission(const Data::MissionKey &key);
    //void requestHome(const int &systemID);
    //void receivedMissionHome(const mace_home_position_t &systemHome);


    Data::ControllerCommsState getCommsState() const
    {
        return this->currentCommsState;
    }
private:
    void clearPreviousTransmit();

private:
    int systemID;
    int transmittingID;

    Timer mTimer;
    bool mToExit;
    int currentRetry;
    int maxRetries;
    int responseTimeout;

private:
    std::shared_ptr<spdlog::logger> mLog;

    MissionController_Interface *m_CB;
    PreviousTransmissionBase<commsItemEnum> *prevTransmit;

    Helper_MissionCOMMStoMACE helperCOMMStoMACE;
    Helper_MissionMACEtoCOMMS helperMACEtoCOMMS;


    Data::ControllerCommsState currentCommsState;

    MissionItem::MissionList missionList;
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

#endif // MISSION_CONTROLLER_EXTERNALLINK_H
