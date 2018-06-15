#ifndef BOUNDARY_KEY_H
#define BOUNDARY_KEY_H

#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "boundary_type.h"

namespace BoundaryItem {

class BoundaryKey
{
public:
    BoundaryKey();
    BoundaryKey(const int &systemID, const int &creatorID, const BOUNDARYTYPE &boundaryType);
    BoundaryKey(const BoundaryKey &obj);
public:
    int m_systemID;
    int m_creatorID;

    //!
    //! \brief m_boundaryID This descriptor is a unique identifier for the vehicle to reference to
    //! the transmitter of the information in the exchange. This should help handle ack events
    //! to stations that may/may not have lost sync with vehicle. More robust methods should be
    //! investigated in the future such as timestamping etc.
    //!
    uint64_t m_boundaryID;

    BOUNDARYTYPE m_boundaryType;


    BoundaryKey& operator =(const BoundaryKey &rhs);

    bool operator< (const BoundaryKey &rhs) const;

    bool operator== (const BoundaryKey &rhs) const;

    bool operator!= (const BoundaryKey &rhs) const;

    friend std::ostream& operator<<(std::ostream& os, const BoundaryKey& t);

    friend class BoundaryKeyHasher;
};

} //end of namespace Data

namespace std
{
template <>
struct hash<BoundaryItem::BoundaryKey>
{
    std::size_t operator()(const BoundaryItem::BoundaryKey& k) const
    {
        using std::size_t;
        using std::hash;
        using std::string;

        return ((hash<int>()(k.m_systemID))
                 ^ (hash<int>()(k.m_creatorID))
                 ^ (hash<int>()((int)k.m_boundaryType)));
    }
};
}
#endif // BOUNDARY_KEY_H
