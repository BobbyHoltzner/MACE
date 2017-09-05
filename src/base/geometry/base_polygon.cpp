#include "base_polygon.h"

namespace mace{
namespace geometry {

template<class T>
PolygonBase<T>::PolygonBase()
{

}

template<class T>
PolygonBase<T>::PolygonBase(const std::vector<T> &vector)
{

}

template <class T>
PolygonBase<T>::appendVertex(const T &vertex)
{
    m_vertex.push_back(vertex);
}

template<class T>
PolygonBase<T>::removeVertex(const int &index)
{

}

template<class T>
size_t PolygonBase<T>::polygonSize() const
{
    return m_vertex.size();
}

template<class T>
void PolygonBase<T>::clearPolygon()
{
    m_vertex.clear();
}

} //end of namepsace geometry
} //end of namespace mace
