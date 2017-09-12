#ifndef BOUNDED_2D_GRID_H
#define BOUNDED_2D_GRID_H

#include "dynamic_2D_grid.h"

#include "base/pose/cartesian_position_2D.h"
#include "base/geometry/polygon_2dc.h"

namespace mace {
namespace maps {

template <class T>
class Bounded2DGrid : public Dynamic2DGrid
{
public:
    Bounded2DGrid(const double &x_min = -10.0, const double &x_max = 10.0,
                   const double &y_min = -10.0, const double &y_max = 10.0,
                   const double &x_res = 0.5, const double &y_res = 0.5,
                   const T *fill_value);

    void setBoundingPolygon(const geometry::Polygon_2DC &polygon);


protected:
    geometry::Polygon_2DC boundary;

};

} //end of namespace maps
} //end of namespace mace

#endif // BOUNDED_2D_GRID_H
