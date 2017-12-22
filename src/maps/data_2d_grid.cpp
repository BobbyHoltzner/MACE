#include "data_2d_grid.h"

namespace mace{
namespace maps {

template <class T>
Data2DGrid<T>::Data2DGrid(const double &x_min, const double &x_max,
                       const double &y_min, const double &y_max,
                       const double &x_res, const double &y_res,
                       const T *fill_value):
    BaseGridMap(x_max - x_min, y_max - y_min, x_res, y_res,
                pose::CartesianPosition_2D((x_max-x_min)/2,(y_max-y_min)/2))
{
    sizeGrid(fill_value);
}

template <class T>
void Data2DGrid<T>::sizeGrid(const T *fill_value)
{
    // Cells memory:
    if (fill_value)
    {
        m_defaultFill = *fill_value;
        m_dataMap.assign(xSize * ySize, *fill_value);
    }
    else
        m_dataMap.resize(xSize * ySize);
}

template <class T>
void Data2DGrid<T>::clear()
{
    m_dataMap.clear();
    m_dataMap.resize(xSize*ySize);
    fill(m_defaultFill);
}

template <class T>
void Data2DGrid<T>::fill(const T &value)
{
    for (typename std::vector<T>::iterator it = m_dataMap.begin(); it != m_dataMap.end(); ++it)
        *it = value;
}

template <class T>
T* Data2DGrid<T>::getCellByIndex(const unsigned int &index)
{
    if (index > (this->getNodeCount() - 1))
        return nullptr;
    else
        return &m_dataMap[index];
}

template <class T>
T* Data2DGrid<T>::getCellByPosIndex(const unsigned int &xIndex, const unsigned int &yIndex) const
{
    if (xIndex >= xSize || yIndex >= ySize)
        return nullptr;
    else
        return &m_dataMap[xIndex + yIndex * xSize];
}

template <class T>
T* Data2DGrid<T>::getCellByPos(const double &x, const double &y) const
{
    int cx = indexFromXPos(x);
    int cy = indexFromYPos(y);
    if (cx < 0 || cx >= static_cast<int>(xSize)) return nullptr;
    if (cy < 0 || cy >= static_cast<int>(ySize)) return nullptr;
    return &m_dataMap[cx + cy * xSize];
}


} //end of namespace maps
} //end of namespace mace
