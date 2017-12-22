#include "double_2d_grid.h"

namespace mace{
namespace maps {

Double2DGrid::Double2DGrid(const double &x_min, const double &x_max,
                       const double &y_min, const double &y_max,
                       const double &x_res, const double &y_res,
                       const double *fill_value):
    Data2DGrid(x_min, x_max, y_min, y_max, x_res, y_res, fill_value)
{

}

}
}
