#include "mission_controller_mavlink.h"

namespace DataInterface_MAVLINK {

MissionController_MAVLINK::MissionController_MAVLINK():
    mToExit(false)
{

}

void MissionController_MAVLINK::run()
{

}

void MissionController_MAVLINK::forceCallback()
{
    if(this->m_p)
    {
        mavlink_message_t msg;
        m_CBMisMsg(m_p,msg);
    }
}

} //end of namespace DataInterface_MAVLINK
