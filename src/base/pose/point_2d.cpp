#include "point_2d.h"

namespace mace {
namespace pose {

Point2D::Point2D()
{

}

Point2D::~Point2D()
{

}

Point2D::Point2D(const Point2D &copy)
{
    this->x = copy.x;
    this->posXFlag = copy.posXFlag;

    this->y = copy.y;
    this->posYFlag = copy.posYFlag;
}

Point2D::Point2D(const double &x, const double &y)
{
    this->set2DPosition(x,y);
}

void Point2D::set2DPosition(const Point2D &point2D)
{
    this->set2DPosition(point2D.getX(),this->getY());
}

void Point2D::set2DPosition(const double &x, const double &y)
{
    this->setX(x);
    this->setY(y);
}

} //end of namespace pose
} //end of namespace mace
