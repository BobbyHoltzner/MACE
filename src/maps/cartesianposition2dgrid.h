#ifndef CARTESIANPOSITION2DGRID_H
#define CARTESIANPOSITION2DGRID_H

#include "data_2d_grid.h"
#include "base/pose/base_position.h"
#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace maps {

using namespace pose;

class CartesianPosition2DGrid : public Data2DGrid<Position<CartesianPosition_2D>>
{
public:
    CartesianPosition2DGrid(const double &x_min = -10.0, const double &x_max = 10.0,
                  const double &y_min = -10.0, const double &y_max = 10.0,
                  const double &x_res = 0.5, const double &y_res = 0.5,
                   const Position<CartesianPosition_2D> *fill_value = nullptr);

};

}
}

#endif // CARTESIANPOSITION2DGRID_H
