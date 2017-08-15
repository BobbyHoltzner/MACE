#include "mission_ack.h"

namespace MissionItem {

MissionACK::MissionACK(const int &ID, const MISSION_RESULT &ack, const MissionKey &key, const MISSIONSTATE &state):
    m_SystemID(ID), result(ack), refKey(key), newState(state)
{

}

}
