#ifndef GRID_MAP_ITERATOR_H
#define GRID_MAP_ITERATOR_H

#include <stddef.h>

#include "maps/dynamic_2D_grid.h"

namespace mace{
namespace maps {

class GridMapIterator
{
public:
    GridMapIterator(const Dynamic2DGrid* map);

    GridMapIterator(const GridMapIterator* copy);

    GridMapIterator& operator =(const GridMapIterator &rhs);

    bool operator !=(const GridMapIterator &rhs) const;

    const int operator *() const;

    virtual GridMapIterator& operator ++();

    GridMapIterator begin() const;

    GridMapIterator end() const;

    bool isPastEnd() const;

private:

    size_t currentIndex;
    size_t mapSize;

    bool pastEnd;
};

} //end of namepsace mace
} //end of namespace maps


#endif // GRID_MAP_ITERATOR_H
