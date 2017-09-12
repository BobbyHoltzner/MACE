#include "bounded_2D_grid.h"

namespace mace {
namespace maps {

Bounded2DGrid::Bounded2DGrid(const double &x_min, const double &x_max,
                             const double &y_min, const double &y_max,
                             const double &x_res, const double &y_res,
                             const T *fill_value):
    Dynamic2DGrid(x_min, x_max, y_min, y_max, x_res, y_res, fill_value)
{

}

void Bounded2DGrid::setBoundingPolygon(const geometry::Polygon_2DC &polygon)
{
    this->boundary = polygon;

}

} //end of namespace maps
} //end of namespace mace

