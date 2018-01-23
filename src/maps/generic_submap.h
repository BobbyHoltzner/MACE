#ifndef GENERIC_SUBMAP_H
#define GENERIC_SUBMAP_H

#include "base_grid_map.h"

#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace maps {

class GenericSubMap
{
public:
    GenericSubMap(const BaseGridMap* map, const pose::CartesianPosition_2D &position, const double xSize, const double ySize, bool &valid);

private:
    const BaseGridMap* parentMap;

    size_t startIndex;

    size_t subMapSize;

    pose::CartesianPosition_2D center;

    double yLength;

    double xLength;

    size_t requestedIndex;

};

} //end of namespace maps
} //end of namepsace mace

#endif // GENERIC_SUBMAP_H
