#include "polygon_2dc.h"

namespace mace{
namespace geometry {

Polygon_2DC::Polygon_2DC(const std::string &descriptor):
    PolygonBase(descriptor)
{

}

bool Polygon_2DC::contains(const double &x, const double &y, const bool &onLineCheck)
{
    const size_t num = this->m_vertex.size();

    if (num < 3)
        return false;

    if(onLineCheck == true)
    {
        for(size_t i = 0; (i+1) < num; i++)
        {
            if((i+1) < num){
                if(isOnLine(m_vertex[i],m_vertex[i+1],x,y))
                    return true;
            }
        }

        /* this condition checks the last vertice to the first
         * this was moved outside the for loop to avoid checking
         * this condition at every vertice
         */

        if(isOnLine(m_vertex[num - 1],m_vertex[0],x,y))
            return true;
    }
    return false;


    int counter = 0;
    for(size_t i = 0; i < num; i++)
    {
        if(m_vertex[i].getYPosition() <= y)
        {
            if(m_vertex[(i+1)%num].getYPosition() > y)
            {
                double value = isLeftOfInf(m_vertex[i],m_vertex[(i+1)%num],x,y);
                if(value > 0)
                    ++counter;
                else
                    return true;
            }
        }
        else{
            if(m_vertex[(i+1)%num].getYPosition() <= y)
            {
                double value = isLeftOfInf(m_vertex[i],m_vertex[(i+1)%num],x,y);
                if(value < 0)
                    --counter;
                else
                    return true;
            }
        }
    }

    return counter != 0;
}

bool Polygon_2DC::contains(const Position<CartesianPosition_2D> &point, const bool &onLineCheck)
{
    return contains(point.getXPosition(), point.getYPosition(),onLineCheck);
}

void Polygon_2DC::updateBoundingBox()
{
    bool firstExe = true;
    double maxXVal, minXVal, maxYVal, minYVal;
    if (m_vertex.size() >= 3)
    {
        if(firstExe)
        {
            m_xMax = m_vertex[0].getXPosition();
            m_xMin = m_xMax;
            m_yMax = m_vertex[0].getYPosition();
            m_yMin = m_yMax;
        }

        const size_t num = this->m_vertex.size();
        for(size_t i = 1; i < num; i++)
        {
            if(m_vertex[i].getXPosition() > maxXVal)
                m_xMax = m_vertex[i].getXPosition();
            else if(m_vertex[i].getXPosition() < minXVal)
                m_xMin = m_vertex[i].getXPosition();
            if(m_vertex[i].getYPosition() > maxYVal)
                m_yMax = m_vertex[i].getYPosition();
            else if(m_vertex[i].getYPosition() < minYVal)
                m_yMin = m_vertex[i].getYPosition();
        }
    }
}

void Polygon_2DC::getBoundingValues(double &xMin, double &yMin, double &xMax, double &yMax) const
{
    xMin = m_xMin;
    yMin = m_yMin;
    xMax = m_xMax;
    yMax = m_yMax;
}

Polygon_2DC Polygon_2DC::getBoundingRect() const
{
    Polygon_2DC polygon("Bounding Polygon");

    Position<CartesianPosition_2D> LL("Lower Left",m_xMin,m_yMin);
    Position<CartesianPosition_2D> UL("Upper Left",m_xMin,m_yMax);
    Position<CartesianPosition_2D> UR("Upper Right",m_xMax,m_yMax);
    Position<CartesianPosition_2D> LR("Lower Right",m_xMax,m_yMin);

    polygon.appendVertex(LL);
    polygon.appendVertex(UL);
    polygon.appendVertex(UR);
    polygon.appendVertex(LR);

    return polygon;
}


Position<pose::CartesianPosition_2D> Polygon_2DC::getCenter() const
{
    Position<CartesianPosition_2D> center("Center");
    size_t size = polygonSize();
    for (size_t i = 0; i < size; i++)
    {
        center += m_vertex[i];
    }
    center.setXPosition(center.getXPosition()/size);
    center.setYPosition(center.getYPosition()/size);
    return center;
}


}
}
