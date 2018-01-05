#ifndef MISSION_DOWNLOAD_CONTROLLER_H
#define MISSION_DOWNLOAD_CONTROLLER_H
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

#include "generic_controller.h"

//typedef void(*CallbackFunctionPtr_MisCount)(void*, mace_message_t &);

using namespace DataInterface_MACE;

namespace ExternalLink {


class MissionDownloadInterface : public GenericControllerInterface
{
public:
    virtual void ReceivedMission(const MissionItem::MissionList &list) = 0;
};


class MissionDownloadController : public GenericControllerSpecalizedCallback<MissionDownloadInterface>
{
private:

    struct MissionRequestStruct
    {
        MaceCore::ModuleCharacteristic requester;
        MissionItem::MissionList missionList;
        int currentActiveTransmission;
    };

    std::shared_ptr<spdlog::logger> mLog;

    std::map<MissionItem::MissionKey, MissionRequestStruct> m_MissionsBeingFetching;

public:
    MissionDownloadController(MissionDownloadInterface *cb, int linkChan);

    ~MissionDownloadController() {
        std::cout << "Destructor on the mavlink mission controller" << std::endl;
        mToExit = true;
    }

    //!
    //! \brief Receive a message for the controller
    //! \param message Message to receive
    //! \return True if action was taken, false if this module didnt' care about message
    //!
    virtual bool ReceiveMessage(const mace_message_t* message);


    void receivedMissionCount(const mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender);


    //!
    //! \brief recievedMissionItem is called when the instance receives a mission item. This message should be heard in response
    //! to emitting the mission_request_item message indicating a key and appropriate index of interest.
    //! \param missionItem object containing the information about the mission item at an appropriate index.
    //!
    void recievedMissionItem(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender);


    //!
    //! \brief requestMission transmits a request to the appropriate system as dictated by the key for the mission
    //! associated with it. This function will initiate the controller thread to make up to N attempts until a mission
    //! count or acknowledgement is heard from the target system.
    //! \param key object describing the mission that this system is interesting in knowing about.
    //!
    void requestMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &target, const MaceCore::ModuleCharacteristic &sender);


    void requestCurrentMission(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>());


};

}

#endif // MISSION_DOWNLOAD_CONTROLLER_H
