#ifndef BOUNDED_2D_GRID_H
#define BOUNDED_2D_GRID_H

#include <unordered_map>

#include "base/pose/cartesian_position_2D.h"
#include "base/geometry/polygon_2dc.h"

#include "dynamic_2D_grid.h"


namespace mace {
namespace maps {

class Test
{
public:
    Test() = default;

    pose::CartesianPosition_2D pos;
};

class Bounded2DGrid : public Dynamic2DGrid<Test>
{
public:
    Bounded2DGrid(const double &x_min = -1.0, const double &x_max = 1.0,
                  const double &y_min = -1.0, const double &y_max = 1.0,
                  const double &x_res = 0.5, const double &y_res = 0.5,
                   const Test *fill_value = nullptr);

    Bounded2DGrid(const geometry::Polygon_2DC &boundingPolygon,
                  const double &x_res = 0.5, const double &y_res = 0.5,
                  const Test *fill_value = nullptr);

    std::vector<Test*> setBoundingPolygon(const geometry::Polygon_2DC &polygon);

    std::vector<Test*> getBoundedDataPoints() const;

private:
    void clearData();

protected:
    geometry::Polygon_2DC m_boundary;
    std::vector<Test*> m_constrainedData;
};

} //end of namespace maps
} //end of namespace mace

#endif // BOUNDED_2D_GRID_H
