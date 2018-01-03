#ifndef DATA_2D_GRID_H
#define DATA_2D_GRID_H

#include <iostream>
#include <vector>

#include "base_grid_map.h"

namespace mace {
namespace maps {

template <class T>
class Data2DGrid : public BaseGridMap
{
public:
    Data2DGrid(const double &x_min, const double &x_max,
                           const double &y_min, const double &y_max,
                           const double &x_res, const double &y_res,
                           const T *fill_value):
        BaseGridMap(x_max - x_min, y_max - y_min, x_res, y_res,
                    pose::CartesianPosition_2D((x_max-x_min)/2,(y_max-y_min)/2))
    {
        sizeGrid(fill_value);
    }

    virtual ~Data2DGrid() = default;

    //!
    //! \brief clear
    //!
    void clear()
    {
        m_dataMap.clear();
        m_dataMap.resize(xSize*ySize);
        fill(m_defaultFill);
    }

    //!
    //! \brief fill
    //! \param value
    //!
    void fill(const T &value)
    {
        for (typename std::vector<T>::iterator it = m_dataMap.begin(); it != m_dataMap.end(); ++it)
            *it = value;
    }

    T* getCellByIndex(const unsigned int &index)
    {
        if (index > (this->getNodeCount() - 1))
            return nullptr;
        else
            return &m_dataMap[index];
    }


    //!
    //! \brief getCellByPos
    //! \param x
    //! \param y
    //! \return
    //!
    T* getCellByPos(const double &x, const double &y) const
    {
        int cx = indexFromXPos(x);
        int cy = indexFromYPos(y);
        if (cx < 0 || cx >= static_cast<int>(xSize)) return nullptr;
        if (cy < 0 || cy >= static_cast<int>(ySize)) return nullptr;
        return &m_dataMap[cx + cy * xSize];
    }

    //!
    //! \brief getCellByIndex
    //! \param xIndex
    //! \param yIndex
    //! \return
    //!
    T* getCellByPosIndex(const unsigned int &xIndex, const unsigned int &yIndex) const
    {
        if (xIndex >= xSize || yIndex >= ySize)
            return nullptr;
        else
            return &m_dataMap[xIndex + yIndex * xSize];
    }


    std::vector<T> getDataMap() const
    {
        return this->m_dataMap;
    }
protected:
    void sizeGrid(const T *fill_value)
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

protected:
    //!
    //! \brief m_dataMap
    //!
    std::vector<T> m_dataMap;

    //!
    //! \brief m_defaultFill
    //!
    T m_defaultFill;

};

} //end of namespace maps
} //end of namespace mace

#endif // DATA_2D_GRID_H
