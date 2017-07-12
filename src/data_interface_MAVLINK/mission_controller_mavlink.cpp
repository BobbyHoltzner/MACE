#include "mission_controller_mavlink.h"

namespace DataInterface_MAVLINK {

MissionController_MAVLINK::MissionController_MAVLINK():
    mToExit(false), currentCommsState(NEUTRAL), currentRetry(0), maxRetries(5), responseTimeout(1000)
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
    mTimer.start();
    currentRetry = 0;
}

void MissionController_MAVLINK::run()
{
    while(true)
    {
        double timeElapsed = mTimer.elapsedMilliseconds();
        if(mToExit == true) {
            break;
        }
        if(timeElapsed > responseTimeout)
        {
            currentRetry++;
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
    mTimer.stop();
    currentRetry = 0;
}

void MissionController_MAVLINK::receivedMissionACK(const mavlink_mission_ack_t &missionAck)
{
    mTimer.stop();
    currentRetry = 0;
}

void MissionController_MAVLINK::recievedMissionItem(const mavlink_mission_item_t &missionItem)
{
    mTimer.stop();
    currentRetry = 0;
}

void MissionController_MAVLINK::forceCallback()
{

}

} //end of namespace DataInterface_MAVLINK
