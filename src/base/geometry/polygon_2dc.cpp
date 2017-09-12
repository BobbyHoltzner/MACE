#include "polygon_2dc.h"

namespace mace{
namespace geometry {

Polygon_2DC::Polygon_2DC(const std::string &descriptor):
    PolygonBase(descriptor)
{

}

bool Polygon_2DC::contains(const double &x, const double &y, const bool &onLineCheck)
{
    if (m_vertex.size() < 3)
        return false;

    const size_t num = this->m_vertex.size();

    if(onLineCheck == true)
    {
        for(size_t i = 0; i < num; i++)
        {
            if(isOnLine(m_vertex[i],m_vertex[i+1],x,y))
                return true;
        }
    }


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


Polygon_2DC Polygon_2DC::getBoundingRect() const
{
    bool firstExe = true;
    double maxXVal, minXVal, maxYVal, minYVal;
    Polygon_2DC polygon("Bounding Polygon");

    if (m_vertex.size() >= 3)
    {
        if(firstExe)
        {
            maxXVal = m_vertex[0].getXPosition();
            minXVal = maxXVal;
            maxYVal = m_vertex[0].getYPosition();
            minYVal = maxYVal;
        }

        const size_t num = this->m_vertex.size();
        for(size_t i = 1; i < num; i++)
        {
            if(m_vertex[i].getXPosition() > maxXVal)
                maxXVal = m_vertex[i].getXPosition();
            else if(m_vertex[i].getXPosition() < minXVal)
                minXVal = m_vertex[i].getXPosition();
            if(m_vertex[i].getYPosition() > maxYVal)
                maxYVal = m_vertex[i].getYPosition();
            else if(m_vertex[i].getYPosition() < minYVal)
                minYVal = m_vertex[i].getYPosition();
        }


        Position<CartesianPosition_2D> LL("Lower Left",minXVal,minYVal);
        Position<CartesianPosition_2D> UL("Upper Left",minXVal,maxYVal);
        Position<CartesianPosition_2D> UR("Upper Right",maxXVal,maxYVal);
        Position<CartesianPosition_2D> LR("Lower Right",maxXVal,minYVal);

        polygon.appendVertex(LL);
        polygon.appendVertex(UL);
        polygon.appendVertex(UR);
        polygon.appendVertex(LR);
    }

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
