#ifndef DOUBLE_2D_GRID_H
#define DOUBLE_2D_GRID_H

#include "data_2d_grid.h"

namespace mace {
namespace maps {

class Double2DGrid : public Data2DGrid<double>
{
public:
    Double2DGrid(const double &x_min = -10.0, const double &x_max = 10.0,
                  const double &y_min = -10.0, const double &y_max = 10.0,
                  const double &x_res = 0.5, const double &y_res = 0.5,
                   const double *fill_value = nullptr);

};

}
}
#endif // DOUBLE_2D_GRID_H
