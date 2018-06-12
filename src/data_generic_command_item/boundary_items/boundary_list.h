#ifndef BOUNDARY_LIST_H
#define BOUNDARY_LIST_H

#include "common/common.h"

#include "base/geometry/polygon_2DC.h"

#include "boundary_key.h"

using namespace mace::pose;
using namespace std;

MACE_CLASS_FORWARD(BoundaryList);

namespace BoundaryItem {

typedef std::pair<int, BOUNDARYTYPE> BoundaryMapPair;

}

namespace std {

  template <>
  struct hash<BoundaryItem::BoundaryMapPair>
  {
    std::size_t operator()(const BoundaryItem::BoundaryMapPair& k) const
    {
      using std::size_t;
      using std::hash;
      using std::string;

      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

      return ((hash<int>()(k.first)
               ^ (hash<int>()((int)k.second) << 1)) >> 1);
    }
  };

}

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

    BOUNDARYTYPE getBoundaryType() const{
        return this->boundaryKey.m_boundaryType;
    }

    mace::geometry::Polygon_2DC getBoundary() {
        return boundingPolygon;
    }

    void setBoundary(const mace::geometry::Polygon_2DC &boundary) {
        boundingPolygon = boundary;
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

public:
    mace::geometry::Polygon_2DC boundingPolygon;

private:
    BoundaryKey boundaryKey;

public:
    friend std::ostream& operator<<(std::ostream& os, const BoundaryList& t);
};

} //end of namespace MissionItem
#endif // BOUNDARY_LIST_H
