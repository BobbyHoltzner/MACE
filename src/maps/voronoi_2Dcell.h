#ifndef VORONOI_CELL_H
#define VORONOI_CELL_H

#include "base/geometry/polygon_2dc.h"

namespace mace{
namespace maps{

using namespace pose;
using namespace geometry;

template <class DATA>
class Voronoi_Cell
{
public:
    Voronoi_Cell() = default;

    Voronoi_Cell(const Polygon_2DC &boundary);

    Position<CartesianPosition_2D> getCentroid() const;

private:
    Polygon_2DC boundary;
    std::vector<DATA> data;
};

} //end of namespace maps
} //end of namespace mace

#endif // VORONOI_CELL_H
