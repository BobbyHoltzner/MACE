#ifndef BASE_GRID_MAP_H
#define BASE_GRID_MAP_H

#include <math.h>

#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace maps {

class BaseGridMap
{
public:

    //!
    //! \brief BaseGridMap
    //! \param x_min
    //! \param x_max
    //! \param y_min
    //! \param y_max
    //! \param x_res
    //! \param y_res
    //! \param fill_value
    //!
    BaseGridMap(const double &x_length = 10.0, const double &y_length = 10.0,
                const double &x_res = 0.5, const double &y_res = 0.5,
                const pose::CartesianPosition_2D &position);

    //!
    //! \brief ~Dynamic2DGrids
    //!
    virtual ~BaseGridMap()
    {

    }


    void updateGridSize(const double &x_length = 10.0, const double &y_length = 10.0,
                     const double &x_res = 0.5, const double &y_res = 0.5);

    void updatePosition(const pose::CartesianPosition_2D &position);

    //!
    //! \brief indexFromXPos
    //! \param x
    //! \return
    //!
    int indexFromXPos(const double &x) const
    {
        return static_cast<int>(round((x - m_xMin) / m_xResolution));
    }

    void getPositionFromIndex(const unsigned int &index, double &x, double &y)
    {
        int yIndex = floor(index/m_xSize);
        y = m_yMin + yIndex * m_yResolution;

        int xIndex = (index % m_xSize);
        x = m_xMin + xIndex * m_xResolution;

    }

    //!
    //! \brief indexFromYPos
    //! \param y
    //! \return
    //!
    int indexFromYPos(const double &y) const
    {
        return static_cast<int>(round((y - m_yMin) / m_yResolution));
    }

    //!
    //! \brief indexFromPos
    //! \param x
    //! \param y
    //! \return
    //!
    int indexFromPos(const double &x, const double &y) const
    {
        return indexFromXPos(x) + indexFromYPos(y) * this->m_xSize;
    }

    //!
    //! \brief getSizeX
    //! \return
    //!
    size_t getSizeX() const
    {
        return this->m_xSize;
    }

    //!
    //! \brief getSizeY
    //! \return
    //!
    size_t getSizeY() const
    {
        return this->m_ySize;
    }

    //!
    //! \brief getXMin
    //! \return
    //!
    double getXMin() const
    {
        return this->m_xMin;
    }

    //!
    //! \brief getYMin
    //! \return
    //!
    double getYMin() const
    {
        return this->m_yMin;
    }

    //!
    //! \brief getXMax
    //! \return
    //!
    double getXMax() const
    {
        return this->m_xMax;
    }

    //!
    //! \brief getYMax
    //! \return
    //!
    double getYMax() const
    {
        return this->m_yMax;
    }

    //!
    //! \brief getXResolution
    //! \return
    //!
    double getXResolution() const
    {
        return this->m_xResolution;
    }

    //!
    //! \brief getYResolution
    //! \return
    //!
    double getYResolution() const
    {
        return this->m_yResolution;
    }

    unsigned int getNodeCount() const
    {
        return m_dataMap.size();
    }

protected:
    pose::CartesianPosition_2D position; //!< Position of the map relative to the grid frame

    double xMin, yMin; //!< Description of members
    double xMax, yMax; //!< Description of members
    double xResolution, yResolution; //!< Description of members
    size_t xSize, ySize; //!< Description of members
};

} //end of namespace maps
} //end of namespace mace

#endif // BASE_GRID_MAP_H
