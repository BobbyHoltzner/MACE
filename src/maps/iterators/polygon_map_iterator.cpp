#include "polygon_map_iterator.h"


namespace mace {
namespace maps {

PolygonMapIterator::PolygonMapIterator(const BaseGridMap *map, const geometry::Polygon_2DC &polygon)
{

}

PolygonMapIterator::PolygonMapIterator(const PolygonMapIterator *copy)
{

}

void PolygonMapIterator::determineSubmap(const geometry::Polygon_2DC &polygon, size_t startIndex, size_t mapSize) const
{
    //we have to first make sure the polygon stays inside the available dimensions of the map

}

} //end of namespace maps
} //end of namespace mace
