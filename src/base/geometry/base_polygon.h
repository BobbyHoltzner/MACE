#ifndef BASE_POLYGON_H
#define BASE_POLYGON_H

#include <vector>
#include <stdlib.h>

#include "geometry_helper.h"

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"

namespace mace{
namespace geometry{

template <class T>
class PolygonBase
{
public:
    PolygonBase(const std::string &descriptor = "Polygon"):
        name(descriptor)
    {

    }

    PolygonBase(const std::vector<T> &vector, const std::string &descriptor = "Polygon"):
        name(descriptor)
    {
        //this->clearPolygon(); we should not have to call this case since this is in the constructer
        m_vertex = vector;
        updateBoundingBox();
    }

    PolygonBase(const PolygonBase &copy)
    {
        name = copy.name;
        this->replaceVector(copy.m_vertex);
    }

    void appendVertex(const T &vertex)
    {
        m_vertex.push_back(vertex);
        updateBoundingBox();
    }

    void removeVertex(const int &index);

    void replaceVector(const std::vector<T> &vector)
    {
        this->clearPolygon();
        m_vertex = vector;
        updateBoundingBox();
    }

    void clearPolygon()
    {
        m_vertex.clear();
        m_vertex.shrink_to_fit();
    }

    size_t polygonSize() const
    {
        return m_vertex.size();
    }

    //!
    //! \brief getVector
    //! \return
    //!
    std::vector<T> getVector()
    {
        return m_vertex;
    }

    T at(const int &index)
    {
        return m_vertex[index];
    }

protected:
    virtual void updateBoundingBox()
    {

    }

protected:
    std::string name;
    std::vector<T> m_vertex;
};

} //end of namepsace geometry
} //end of namespace mace
#endif // BASE_POLYGON_H
