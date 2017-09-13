#include "bounded_2D_grid.h"

namespace mace {
namespace maps {

Bounded2DGrid::Bounded2DGrid(const double &x_min, const double &x_max,
                             const double &y_min, const double &y_max,
                             const double &x_res, const double &y_res,
                             const Test *fill_value):
    Dynamic2DGrid(x_min, x_max, y_min, y_max, x_res, y_res, fill_value)
{

}

Bounded2DGrid::Bounded2DGrid(const geometry::Polygon_2DC &boundingPolygon,
                             const double &x_res, const double &y_res,
                             const Test *fill_value):
    Dynamic2DGrid(boundingPolygon.getXMin(),boundingPolygon.getXMin(),
                  boundingPolygon.getXMin(),boundingPolygon.getXMin(),
                  x_res,y_res,fill_value)
{
    setBoundingPolygon(boundingPolygon);
}

std::vector<Test*> Bounded2DGrid::setBoundingPolygon(const geometry::Polygon_2DC &polygon)
{
    this->m_boundary = polygon;
    unsigned int size = getNodeCount();

    for(unsigned int i = 0; i < size; i++)
    {
        double x = 0, y = 0;
        getPositionFromIndex(i,x,y);
        m_dataMap[i].pos.updatePosition(x,y);
        if(m_boundary.contains(x,y,true))
        {
            m_constrainedData.push_back(&m_dataMap[i]);
        }
    }
    return m_constrainedData;
}

std::vector<Test*> Bounded2DGrid::getBoundedDataPoints() const
{
    return this->m_constrainedData;
}

void Bounded2DGrid::clearData()
{
    m_constrainedData.clear();
    m_constrainedData.shrink_to_fit();
}

} //end of namespace maps
} //end of namespace mace

