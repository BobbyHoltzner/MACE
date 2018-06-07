#include "boundary_key.h"

namespace BoundaryItem {

BoundaryKey::BoundaryKey():
    m_systemID(0),m_creatorID(0),m_boundaryType(BOUNDARYTYPE::GENERIC_POLYGON)
{

}

BoundaryKey::BoundaryKey(const int &systemID, const int &creatorID, const BOUNDARYTYPE &boundaryType):
    m_systemID(systemID),m_creatorID(creatorID),m_boundaryType(boundaryType)
{

}

BoundaryKey::BoundaryKey(const BoundaryKey &obj)
{
    this->m_systemID = obj.m_systemID;
    this->m_creatorID =obj.m_creatorID;
    this->m_boundaryType = obj.m_boundaryType;
}

BoundaryKey& BoundaryKey::operator =(const BoundaryKey &rhs)
{
    this->m_systemID = rhs.m_systemID;
    this->m_creatorID =rhs.m_creatorID;
    this->m_boundaryType = rhs.m_boundaryType;
    return *this;
}

bool BoundaryKey::operator <(const BoundaryKey &rhs) const
{
    if(*this == rhs)
        return false;

    if(this->m_systemID > rhs.m_systemID)
        return false;
    if(this->m_creatorID > rhs.m_creatorID)
        return false;
    if(this->m_boundaryType > rhs.m_boundaryType)
        return false;

    return true;
}

bool BoundaryKey::operator ==(const BoundaryKey &rhs) const
{
    if(this->m_systemID != rhs.m_systemID)
        return false;
    if(this->m_creatorID != rhs.m_creatorID)
        return false;
    if(this->m_boundaryType != rhs.m_boundaryType)
        return false;
    return true;
}

bool BoundaryKey::operator !=(const BoundaryKey &rhs) const
{
    return !((*this) == rhs);
}

std::ostream& operator<<(std::ostream& os, const BoundaryKey& t)
{
    std::stringstream stream;
    stream << std::fixed << "Boundary Key: System ID " << std::to_string(t.m_systemID)
           << ", Creator ID"<< std::to_string(t.m_creatorID)
           << ", Boundary Type" << BoundaryTypeToString(t.m_boundaryType) << ".";
    os << stream.str();

    return os;
}

} //end of namespace BoundaryItem
