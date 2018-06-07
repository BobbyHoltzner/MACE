#ifndef BOUNDARY_LIST_H
#define BOUNDARY_LIST_H

#include "base/geometry/polygon_2DC.h"

#include "boundary_key.h"

using namespace mace::pose;

namespace BoundaryItem {

class BoundaryList
{
public:
    enum BoundaryListState{
        COMPLETE,
        INCOMPLETE
    };

    struct BoundaryListStatus{
        BoundaryListState state;
        std::vector<int> remainingItems;
    };

public:
    BoundaryList();
    BoundaryList(const int &targetID, const int &generatorID, const BOUNDARYTYPE &boundaryType);
    BoundaryList(const BoundaryList &rhs);

public:
    void initializeBoundary(const int &size);
    void clearQueue();
    void appendVertexItem(const Position<CartesianPosition_2D> &vertexItem);
    void replaceVertexItemAtIndex(const Position<CartesianPosition_2D> &vertexItem, const int &index);

    Position<CartesianPosition_2D> getBoundaryItemAtIndex(const int &index) const;
    int getQueueSize() const;
    BoundaryList::BoundaryListStatus getBoundaryListStatus() const;

public:

    BoundaryKey getBoundaryKey() const{
        return this->boundaryKey;
    }

    void setBoundaryKey(const BoundaryKey &key){
        this->boundaryKey = key;
    }

    void setVehicleID(const int &vehicleID){
        this->boundaryKey.m_systemID = vehicleID;
    }

    int getVehicleID() const{
        return this->boundaryKey.m_systemID;
    }

    void setCreatorID(const int &creatorID){
        this->boundaryKey.m_creatorID = creatorID;
    }

    int getCreatorID() const {
        return this->boundaryKey.m_creatorID;
    }

    void setBoundaryType(const BOUNDARYTYPE &boundaryType){
        this->boundaryKey.m_boundaryType = boundaryType;
    }

    BOUNDARYTYPE getMissionType() const{
        return this->boundaryKey.m_boundaryType;
    }

public:
    BoundaryList& operator = (const BoundaryList &rhs)
    {
        this->boundaryKey = rhs.boundaryKey;
        this->boundingPolygon = rhs.boundingPolygon;
        return *this;
    }

    bool operator == (const BoundaryList &rhs) const{
        if(this->boundaryKey != rhs.boundaryKey){
            return false;
        }
        if(this->boundingPolygon != rhs.boundingPolygon){
            return false;
        }
        return true;
    }

    bool operator != (const BoundaryList &rhs) const{
        return !(*this == rhs);
    }

private:
    BoundaryKey boundaryKey;
    mace::geometry::Polygon_2DC boundingPolygon;

public:
    friend std::ostream& operator<<(std::ostream& os, const BoundaryList& t);
};

} //end of namespace MissionItem
#endif // BOUNDARY_LIST_H
