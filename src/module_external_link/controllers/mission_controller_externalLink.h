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
    virtual void cbiMissionController_TransmitMissionGenericReqList(const mace_mission_request_list_generic_t &request) = 0;
    virtual void cbiMissionController_TransmitMissionReq(const mace_mission_request_item_t &requestItem) = 0;
    virtual void cbiMissionController_TransmitMissionACK(const mace_mission_ack_t &missionACK) = 0;

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

    void updateLogging(const bool &toLog, const std::string &name);

    void connectCallback(MissionController_Interface *cb)
    {
        m_CB = cb;
    }

    void updateIDS(const int &targetSystem, const int &originSystem);

    void run();

    void transmitMission(const int &targetSystem, const std::list<MissionItem::MissionList> &missionQueue);


    //These 3 functions are related to transmitting a mission
    //!
    //! \brief transmitMission function initiates transmitting the MissionList object to a remote instance. This function shal
    //! transmit the mission_count message initiating a write transaction on the receiving instance. This function will initiate the controller thread
    //! to make up to N attempts until a mission_item request or acknowledgement is heard from the target system.
    //! \param targetSystem is the system that this information is getting written to.
    //! \param missionQueue is the object containing the information to be sent to the target system.
    //!
    void transmitMission(const int &targetSystem, const MissionItem::MissionList &missionQueue);

    //!
    //! \brief transmitMissionItem is a lambda function often initiated in response to the appropriate instance receiving
    //! a mission_request_item. This function would interrupt reattempting to send a mission_count message and move the
    //! state machine onto attempting to transmit this item.
    //! \param missionRequest is the object containing the information containing the key and index of the item. If the key
    //! does not match the key contained in this thread's missionList an mission result shall be emitted as response.
    //!
    void transmitMissionItem(const mace_mission_request_item_t &missionRequest);

    //!
    //! \brief receivedMissionACK
    //! \param missionACK
    //!
    void receivedMissionACK(const mace_mission_ack_t &missionACK);

    void receivedMissionCount(const mace_mission_count_t &mission);

    //!
    //! \brief recievedMissionItem is called when the instance receives a mission item. This message should be heard in response
    //! to emitting the mission_request_item message indicating a key and appropriate index of interest.
    //! \param missionItem object containing the information about the mission item at an appropriate index.
    //!
    void recievedMissionItem(const mace_mission_item_t &missionItem);

    //This function creates a generic request of mission type
    //!
    //! \brief requestGenericMission transmits a request to the target system request any information known about
    //! the mission type and state available based on the request. This function will initiate the controller thread
    //! to make up to N attempts until a mission or acknowledgement is heard from the target system.
    //! \param targetSystem represents the ID of the system intended to receive the mission request
    //! \param type enumeration of mission that this system is interested in knowing about
    //! \param state enumeration of the mission that this system is interested in knowing about
    //!
    void requestGenericMission(const int &targetSystem, const MAV_MISSION_TYPE &type, const MAV_MISSION_STATE &currentState);

    //!
    //! \brief requestCurrentMission
    //! \param targetSystem
    //!
    void requestCurrentMission(const int &targetSystem);

    //!
    //! \brief requestMission transmits a request to the appropriate system as dictated by the key for the mission
    //! associated with it. This function will initiate the controller thread to make up to N attempts until a mission
    //! count or acknowledgement is heard from the target system.
    //! \param key object describing the mission that this system is interesting in knowing about.
    //!
    void requestMission(const MissionItem::MissionKey &key);

    Data::ControllerCommsState getCommsState() const
    {
        return this->currentCommsState;
    }
private:
    void clearPreviousTransmit();

    void updateTransmittingJobs();

private:
    int targetID;
    int transmittingID;

    Timer mTimer;
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

    std::map<int, std::list<MissionItem::MissionList>> missionList;

    MissionItem::MissionList missionQueue;

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
