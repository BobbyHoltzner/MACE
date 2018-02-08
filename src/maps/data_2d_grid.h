#ifndef DATA_2D_GRID_H
#define DATA_2D_GRID_H

#include <iostream>
#include <vector>

#include "base_grid_map.h"

#include "base/state_space/state_space.h"

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
                    pose::CartesianPosition_2D((x_max+x_min)/2,(y_max+y_min)/2))
    {
        sizeGrid(fill_value);
    }

    Data2DGrid(const double &x_min, const double &x_max,
               const double &y_min, const double &y_max,
               const double &x_res, const double &y_res,
               const T *fill_value, const pose::CartesianPosition_2D &origin):
        BaseGridMap(x_min, x_max, y_min, y_max, x_res, y_res, origin)
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

    //!
    //! \brief getCellByIndex
    //! \param index
    //! \return
    //!
    T* getCellByIndex(const unsigned int &index)
    {
        if (index > (this->getNodeCount() - 1))
            return nullptr;
        else
            return &m_dataMap[index];
    }

    //!
    //! \brief getCellNeighbors
    //! \param index
    //! \return
    //!
    std::vector<T*> getCellNeighbors(const unsigned int &index)
    {
        std::vector<T*> rtnCells;

        unsigned int indexX, indexY;
        getIndexDecomposed(index,indexX,indexY);

        int startY = (((int)indexY + 1) >= ((int)ySize - 1)) ? (ySize - 1) : (int)indexY + 1;
        int endY = (((int)indexY - 1) >= 0) ? (indexY - 1) : 0;

        for(int i = startY; i >= endY; i--)
        {
            int startX = (((int)indexX - 1) >= 0) ? (indexX - 1) : 0;
            for(int j = startX; j <= (int)indexX + 1; j++)
            {
                if((j > (int)xSize - 1) || ((i == (int)indexY) && (j == (int)indexX)))
                    continue;
                std::cout<<"Getting cell number "<<std::to_string(j + i * xSize)<<std::endl;
                rtnCells.push_back(this->getCellByPosIndex(j,i));
            }
        }
        return rtnCells;
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

    bool findIndex(const T* find, int &index)
    {
        for (int i = 0; i < getSize(); i++)
            if(find == &m_dataMap.at(i))
            {
                index = i;
                return true;
            }
        return false;
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


MACE_CLASS_FORWARD(Data2DGridSpace);

class Data2DGridSpace : public state_space::StateSpace
{
public:
    Data2DGridSpace(Data2DGrid<mace::pose::CartesianPosition_2D>* grid)
    {
        currentGrid = grid;
    }

    double traversalCost(const state_space::State* begin, const state_space::State* end) override
    {
        const mace::pose::CartesianPosition_2D* castBegin = begin->as<mace::pose::CartesianPosition_2D>();
        const mace::pose::CartesianPosition_2D* castEnd = end->as<mace::pose::CartesianPosition_2D>();

        return castBegin->distanceBetween2D(*castEnd);
    }

    double distanceBetween(const mace::state_space::State* lhs, const mace::state_space::State* rhs) const override
    {
        return 1.0;
    }

    std::vector<state_space::State*> getNeighboringStates(const state_space::State* currentState) const override
    {
        const mace::pose::CartesianPosition_2D* castState = currentState->as<mace::pose::CartesianPosition_2D>();
        std::vector<state_space::State*> rtn;
        std::vector<mace::pose::CartesianPosition_2D*> newVector;
        int index = currentGrid->indexFromPos(castState->getXPosition(),castState->getYPosition());
        if(index < currentGrid->getNodeCount() - 1)
            newVector = currentGrid->getCellNeighbors(index);
        for(int i = 0; i < newVector.size(); i++)
            rtn.push_back(newVector.at(i)->as<state_space::State>());
        return rtn;
    }



private:
    Data2DGrid<mace::pose::CartesianPosition_2D>* currentGrid;

};

} //end of namespace maps
} //end of namespace mace

#endif // DATA_2D_GRID_H
