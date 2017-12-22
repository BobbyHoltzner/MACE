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
    Data2DGrid(const double &x_min = -10.0, const double &x_max = 10.0,
                const double &y_min = -10.0, const double &y_max = 10.0,
                const double &x_res = 0.5, const double &y_res = 0.5,
                const T *fill_value = nullptr);

    virtual ~Data2DGrid() = default;

    //!
    //! \brief clear
    //!
    void clear();

    //!
    //! \brief fill
    //! \param value
    //!
    void fill(const T& value);

    T* getCellByIndex(const unsigned int &index);


    //!
    //! \brief getCellByPos
    //! \param x
    //! \param y
    //! \return
    //!
    T* getCellByPos(const double &x, const double &y) const;

    //!
    //! \brief getCellByIndex
    //! \param xIndex
    //! \param yIndex
    //! \return
    //!
    T* getCellByPosIndex(const unsigned int &xIndex, const unsigned int &yIndex) const;

    std::vector<T> getDataMap() const
    {
        return this->m_dataMap;
    }
protected:
    void sizeGrid(const T *fill_value);

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

#include "data_2d_grid.cpp"

#endif // DATA_2D_GRID_H
