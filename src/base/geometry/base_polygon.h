#ifndef BASE_POLYGON_H
#define BASE_POLYGON_H

#include <vector>
#include <stdlib.h>

namespace mace{
namespace geometry{

template <class T>
class PolygonBase
{
public:
    PolygonBase();

    PolygonBase(const std::vector<T> &vector);

    appendVertex(const T &vertex);

    removeVertex(const int &index);

    size_t polygonSize() const;

    void clearPolygon();

private:
    std::vector<T> m_vertex;

};

} //end of namepsace geometry
} //end of namespace mace
#endif // BASE_POLYGON_H
