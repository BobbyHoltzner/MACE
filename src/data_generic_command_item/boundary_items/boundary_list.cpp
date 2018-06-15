#include "boundary_list.h"

#include <exception>

namespace BoundaryItem {

BoundaryList::BoundaryList() :
    boundaryKey(0,0,BOUNDARYTYPE::GENERIC_POLYGON), boundingPolygon(BoundaryTypeToString(BOUNDARYTYPE::GENERIC_POLYGON))
{

}

BoundaryList::BoundaryList(const int &targetID, const int &generatorID, const BOUNDARYTYPE &boundaryType) :
    boundaryKey(targetID,generatorID,boundaryType), boundingPolygon(BoundaryTypeToString(boundaryType) + " for " + std::to_string(targetID))
{

}

BoundaryList::BoundaryList(const BoundaryList &rhs)
{
    this->boundaryKey = rhs.boundaryKey;
    this->boundingPolygon = rhs.boundingPolygon;
}

void BoundaryList::initializeBoundary(const int &size)
{
    boundingPolygon.initializePolygon(size);
}

void BoundaryList::clearQueue()
{
    boundingPolygon.clearPolygon();
}

void BoundaryList::appendVertexItem(const Position<CartesianPosition_2D> &vertexItem)
{
    boundingPolygon.appendVertex(vertexItem);
}

void BoundaryList::replaceVertexItemAtIndex(const Position<CartesianPosition_2D> &vertexItem, const int &index)
{
    boundingPolygon.insertVertexAtIndex(vertexItem, index);
}

Position<CartesianPosition_2D> BoundaryList::getBoundaryItemAtIndex(const int &index) const
{
    Position<CartesianPosition_2D> vertex = boundingPolygon.at(index);
    return vertex;
}

int BoundaryList::getQueueSize() const
{
    return boundingPolygon.polygonSize();
}

BoundaryList::BoundaryListStatus BoundaryList::getBoundaryListStatus() const
{
    BoundaryListState boundaryState = BoundaryListState::COMPLETE;
    std::vector<int> nullItems = boundingPolygon.findUndefinedVertices();
    if(nullItems.size() > 0)
        boundaryState = BoundaryListState::INCOMPLETE;

    BoundaryListStatus boundaryStatus;
    boundaryStatus.state = boundaryState;
    boundaryStatus.remainingItems = nullItems;

    return boundaryStatus;
}


std::ostream& operator<<(std::ostream& os, const BoundaryList& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Boundary List Key" << t.getBoundaryKey()
           <<", Size: " << std::to_string(t.getQueueSize()) << ".";
    os << stream.str();

    return os;
}

}//end of namespace BoundaryItem

