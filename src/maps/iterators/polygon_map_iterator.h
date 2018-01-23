#ifndef POLYGON_MAP_ITERATOR_H
#define POLYGON_MAP_ITERATOR_H

#include <stddef.h>

#include "base/geometry/polygon_2DC.h"
#include "maps/base_grid_map.h"

namespace mace {
namespace maps {

class PolygonMapIterator
{
public:
    PolygonMapIterator(const BaseGridMap *map, const geometry::Polygon_2DC &polygon);

    PolygonMapIterator(const PolygonMapIterator* copy);

    void determineSubmap(const geometry::Polygon_2DC &polygon, size_t startIndex, size_t mapSize) const;

    size_t xLength, yLength;

};

} //end of namespace maps
} //end of namespace mace
#endif // POLYGON_MAP_ITERATOR_H
