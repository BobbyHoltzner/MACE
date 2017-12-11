#include "grid_map_iterator.h"

namespace mace{
namespace maps {


GridMapIterator::GridMapIterator(const Dynamic2DGrid* map)
{

}

GridMapIterator::GridMapIterator(const GridMapIterator *copy)
{

}

GridMapIterator& GridMapIterator::operator =(const GridMapIterator& rhs)
{

}

GridMapIterator& GridMapIterator::operator !=(const GridMapIterator& rhs)
{

}

GridMapIterator& GridMapIterator::operator ++()
{
    size_t index = currentIndex + 1;
    if(index < this->mapSize)
        currentIndex = index;
    else
        isPastEnd() = true;
    return *this;
}

GridMapIterator GridMapIterator::begin() const
{

}

GridMapIterator GridMapIterator::end() const
{

}

bool GridMapIterator::isPastEnd() const
{
    return this->pastEnd;
}



} //end of namepsace mace
} //end of namespace maps

