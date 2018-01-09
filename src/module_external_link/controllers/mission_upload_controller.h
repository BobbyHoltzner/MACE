#ifndef MISSION_UPLOAD_CONTROLLER_H
#define MISSION_UPLOAD_CONTROLLER_H

#include <iostream>
#include <QDate>
#include <unordered_map>
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

using namespace DataInterface_MACE;

namespace ExternalLink {


class MissionUploadInterface : public GenericControllerInterface
{
public:
    virtual bool FetchMissionList(const MissionItem::MissionKey &key, MissionItem::MissionList &list) = 0;
    virtual void ActionForAllCurrentMission(int vehicleID, const std::function<void(MissionItem::MissionList list)> &MissionFunc, const std::function<void(const MaceCore::ModuleCharacteristic &vehicle)> &NoMissionFunc) = 0;
};

class MissionUploadController : public GenericControllerSpecalizedCallback<MissionUploadInterface>
{

private:

    struct ActiveMissionUpload
    {
        MaceCore::ModuleCharacteristic requester;
        MissionItem::MissionList mission;
        int currentActiveTransmission;
    };

private:

    std::shared_ptr<spdlog::logger> mLog;

    std::unordered_map<MissionItem::MissionKey, ActiveMissionUpload, MissionKeyHasher> m_MissionsUploading;

public:
    MissionUploadController(MissionUploadInterface *cb, int linkChan);

    //!
    //! \brief Receive a message for the controller
    //! \param message Message to receive
    //! \return True if action was taken, false if this module didnt' care about message
    //!
    virtual bool ReceiveMessage(const mace_message_t* message);

    void transmitMission(const MissionItem::MissionList &missionQueue, const MaceCore::ModuleCharacteristic &target);

private:



    void transmitMissionItem(const mace_mission_request_item_t &missionRequest, const MaceCore::ModuleCharacteristic &target);

    void receivedMissionACK(const mace_mission_ack_t &missionACK);

};


}

#endif // MISSION_UPLOAD_CONTROLLER_H
