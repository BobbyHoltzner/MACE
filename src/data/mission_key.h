#ifndef MISSION_KEY_H
#define MISSION_KEY_H

#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "mission_type.h"

namespace Data {

class MissionKey
{
public:
    MissionKey();
    MissionKey(const int &systemID, const int &creatorID, const int &missionID, const Data::MissionType &missionType);
    MissionKey(const MissionKey &obj);
public:
    int m_systemID;
    int m_creatorID;

    //!
    //! \brief m_missionID This descriptor is a unique identifier for the vehicle to reference to
    //! the transmitter of the information in the exchange. This should help handle ack events
    //! to stations that may/may not have lost sync with vehicle. More robust methods should be
    //! investigated in the future such as timestamping etc.
    //!
    uint64_t m_missionID;

    Data::MissionType m_missionType;

    void operator =(const MissionKey &rhs);

    bool operator< (const MissionKey &rhs) const;

    bool operator== (const MissionKey &rhs) const;

    bool operator!= (const MissionKey &rhs) const;

    friend std::ostream& operator<<(std::ostream& os, const MissionKey& t);

};


} //end of namespace Data
#endif // MISSION_KEY_H
