#include "mission_controller_mavlink.h"

namespace DataInterface_MAVLINK {

MissionController_MAVLINK::MissionController_MAVLINK():
    mToExit(false), currentCommsState(NEUTRAL)
{
    mTimer.start();
}

void MissionController_MAVLINK::transmitMission()
{
    currentCommsState = TRANSMITTING;
    mavlink_message_t msg;
    mavlink_mission_count_t count;
    count.count = 2;
    count.mission_type = MAV_MISSION_TYPE_MISSION;
    count.target_component = 0;
    count.target_system = 0;
    //prevTransmit = new commsObject<mavlink_mission_count_t>(commsObject::ITEMCOUNT, count);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionCount(count);
}

void MissionController_MAVLINK::requestMission()
{
    currentCommsState = RECEIVING;
}

void MissionController_MAVLINK::run()
{
    while(true)
    {
        double timeElapsed = mTimer.elapsedMilliseconds();
        if(mToExit == true) {
            break;
        }

        switch(currentCommsState)
        {
        case NEUTRAL:
        {
            //This case we should terminate this because there is nothing we should be doing apparently
            mToExit = true;
         break;
        }
        case TRANSMITTING:
        {
            break;
        }
        case RECEIVING:
        {
            break;
        }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void MissionController_MAVLINK::receivedMissionCount(const mavlink_mission_count_t &missionCount)
{

}

void MissionController_MAVLINK::receivedMissionACK(const mavlink_mission_ack_t &missionAck)
{

}

void MissionController_MAVLINK::recievedMissionItem(const mavlink_mission_item_t &missionItem)
{

}

void MissionController_MAVLINK::forceCallback()
{

}

} //end of namespace DataInterface_MAVLINK
