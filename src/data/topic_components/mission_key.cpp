#include "mission_key.h"

namespace Data {

namespace TopicComponents
{

const char TopicComponts_MissionKey_name[] = "missionKey";
const MaceCore::TopicComponentStructure TopicComponts_MissionKey_structure = []{
    return MaceCore::TopicComponentStructure();
}();

MaceCore::TopicDatagram MissionKey::GenerateDatagram() const
{

}

void MissionKey::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{

}

MissionKey::MissionKey()
{

}

MissionKey::MissionKey(const int &systemID, const int &creatorID, const int &missionID, const MISSIONTYPE &missionType)
{

}

MissionKey::MissionKey(const int &systemID, const int &creatorID, const int &missionID, const MISSIONTYPE &missionType, const MISSIONSTATE &missionState)
{

}

MissionKey::MissionKey(const MissionKey &obj)
{

}

MissionKey& MissionKey::operator =(const MissionKey &rhs)
{

}

bool MissionKey::operator< (const MissionKey &rhs) const
{

}

bool MissionKey::operator== (const MissionKey &rhs) const
{

}

bool MissionKey::operator!= (const MissionKey &rhs) const
{

}

} // TopicComponents

} // Data
