#ifndef MISSION_CONTROLLER_MAVLINK_H
#define MISSION_CONTROLLER_MAVLINK_H

#include "mavlink.h"

#include "data/threadmanager.h"

#include <callback_interface_data_mavlink.h>

typedef void(*CallbackFunctionPtr_MisMsg)(void*, mavlink_message_t &);

namespace DataInterface_MAVLINK {

class MissionController_MAVLINK : public Thread
{
public:
    MissionController_MAVLINK();

    ~MissionController_MAVLINK() {
        std::cout << "Destructor on the mavlink mission controller" << std::endl;
        mToExit = true;
    }

    void recievedMissionItem(const mavlink_mission_item_t &missionItem);

    void receivedMissionCount(const mavlink_mission_count_t &missionCount);

    void receivedMissionACK(const mavlink_mission_ack_t &missionAck);

    void connectCallback_TransmitMSG(CallbackFunctionPtr_MisMsg cb, void *p)
    {
        m_CBMisMsg = cb;
        m_p = p;
    }

private:
    bool mToExit;

private:
    CallbackFunctionPtr_MisMsg m_CBMisMsg;
    void *m_p;

};

} //end of namespace DataInterface_MAVLINK

#endif // MISSION_CONTROLLER_MAVLINK_H
