#ifndef DATA_2D_GRID_H
#define DATA_2D_GRID_H

#include <iostream>
#include <vector>

#include "common/common.h"
#include "common/class_forward.h"

#include "base_grid_map.h"
#include "iterators/grid_map_iterator.h"

namespace mace {
namespace maps {


template <class T>
class Data2DGrid : public BaseGridMap
{
public:

    Data2DGrid(const T *fill_value,
               const double &x_min = -10, const double &x_max = 10,
               const double &y_min = -10, const double &y_max = 10,
               const double &x_res = 0.5, const double &y_res = 0.5,
               const pose::CartesianPosition_2D &origin = pose::CartesianPosition_2D()):
        BaseGridMap(x_min,x_max,y_min,y_max,x_res,y_res,origin)
    {
        sizeGrid(fill_value);
    }

    Data2DGrid(const T *fill_value,
               const double &x_length, const double &y_length,
               const double &x_res, const double &y_res,
               const pose::CartesianPosition_2D &origin = pose::CartesianPosition_2D()):
        BaseGridMap(x_length,y_length,x_res,y_res,origin)
    {
        sizeGrid(fill_value);
    }

    Data2DGrid(const Data2DGrid &copy):
        BaseGridMap(copy)
    {
        this->m_defaultFill = copy.getFill();
        this->clear();
        mace::maps::GridMapIterator it(this);
        for(;!it.isPastEnd();++it)
        {
            T* ptr = this->getCellByIndex(*it);
            *ptr = *copy.getCellByIndex(*it);
        }
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

    void updateGridSize(const double &minX, const double &maxX, const double &minY, const double &maxY, const double &x_res, const double &y_res) override
    {
        //First clone this object
        BaseGridMap* clone = new BaseGridMap(*this);

        //update the underlying size structure
        BaseGridMap::updateGridSize(minX,maxX,minY,maxY,x_res,y_res);

        //clear this contents and update with the default values
        this->clear();
        //copy the contents over
    }

    void updateGridSizeByLength(const double &x_length = 10.0, const double &y_length = 10.0,
                                const double &x_res = 0.5, const double &y_res = 0.5) override
    {
        //update the underlying size structure
        BaseGridMap::updateGridSizeByLength(x_length,y_length,x_res,y_res);
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


    T getFill() const
    {
        return this->m_defaultFill;
    }

    T* getCellByIndex(const unsigned int &index)
    {
        if (index > (this->getNodeCount() - 1))
            return nullptr;
        else
            return &m_dataMap[index];
    }

    const T* getCellByIndex(const unsigned int &index) const
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
    T* getCellByPos(const double &x, const double &y)
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

    //!
    //! \brief getCellByPosIndex
    //! \param xIndex
    //! \param yIndex
    //! \return
    //!
    T* getCellByPosIndex(const unsigned int &xIndex, const unsigned int &yIndex)
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

public:
    bool operator == (const Data2DGrid &rhs) const
    {
        if(!BaseGridMap::operator ==(rhs))
            return false;
        if(this->m_defaultFill != rhs.m_defaultFill){
            return false;
        }
        mace::maps::GridMapIterator it(this);
        for(;!it.isPastEnd();++it)
        {
            const T* ptr = this->getCellByIndex(*it);
            if(*ptr != *rhs.getCellByIndex(*it))
                return false;
        }
        return true;
    }

    Data2DGrid& operator = (const Data2DGrid &rhs)
    {
        BaseGridMap::operator ==(rhs);
        this->m_defaultFill = rhs.getFill();
        this->m_dataMap = rhs.getDataMap();
        return *this;
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
