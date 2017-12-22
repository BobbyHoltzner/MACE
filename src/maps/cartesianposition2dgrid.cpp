#include "cartesianposition2dgrid.h"

namespace mace{
namespace maps {

using namespace pose;

CartesianPosition2DGrid::CartesianPosition2DGrid(const double &x_min, const double &x_max,
                       const double &y_min, const double &y_max,
                       const double &x_res, const double &y_res,
                       const Position<CartesianPosition_2D> *fill_value):
    Data2DGrid(x_min, x_max, y_min, y_max, x_res, y_res, fill_value)
{

}

}
}
