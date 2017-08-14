#include "mission_ack.h"

namespace MissionItem {

MissionACK::MissionACK(const int &ID, const MISSION_RESULT &ack, const Data::MissionKey &key, const Data::MissionTXState &state):
    m_SystemID(ID), result(ack), refKey(key), newState(state)
{

}

}
