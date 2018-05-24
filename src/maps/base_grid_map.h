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
                const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D());

    BaseGridMap(const double &x_min, const double &x_max,
                  const double &y_min, const double &y_max,
                  const double &x_res, const double &y_res,
                const pose::CartesianPosition_2D &position);

    BaseGridMap(const BaseGridMap &copy);

    //!
    //! \brief ~Dynamic2DGrids
    //!
    virtual ~BaseGridMap() = default;


    void updatePosition(const pose::CartesianPosition_2D &position);

    void getPositionFromIndex(const unsigned int &index, double &x, double &y) const
    {
        int yIndex = floor(index/xSize);
        y = yMin + yIndex * yResolution;

        int xIndex = (index % xSize);
        x = xMin + xIndex * xResolution;
    }

    void getMinPositionFromIndex(const unsigned int &index, double &x, double &y) const
    {
        getPositionFromIndex(index,x,y);
    }

    void getMaxPositionFromIndex(const unsigned int &index, double &x, double &y) const
    {
        getPositionFromIndex(index,x,y);
        x+=xResolution;
        y+=yResolution;
    }

    //!
    //! \brief indexFromXPos
    //! \param x
    //! \return
    //!
    int indexFromXPos(const double &x) const
    {
        return static_cast<int>(floor((x - xMin) / xResolution));
    }

    //!
    //! \brief indexFromYPos
    //! \param y
    //! \return
    //!
    int indexFromYPos(const double &y) const
    {
        return static_cast<int>(floor((y - yMin) / yResolution));
    }

    //!
    //! \brief indexFromPos
    //! \param x
    //! \param y
    //! \return
    //!
    int indexFromPos(const double &x, const double &y) const
    {
        return indexFromXPos(x) + indexFromYPos(y) * this->xSize;
    }

    //!
    //! \brief getSizeX
    //! \return
    //!
    size_t getSizeX() const
    {
        return this->xSize;
    }

    //!
    //! \brief getSizeY
    //! \return
    //!
    size_t getSizeY() const
    {
        return this->ySize;
    }

    //!
    //! \brief getXMin
    //! \return
    //!
    double getXMin() const
    {
        return this->xMin;
    }

    //!
    //! \brief getYMin
    //! \return
    //!
    double getYMin() const
    {
        return this->yMin;
    }

    //!
    //! \brief getXMax
    //! \return
    //!
    double getXMax() const
    {
        return this->xMax;
    }

    //!
    //! \brief getYMax
    //! \return
    //!
    double getYMax() const
    {
        return this->yMax;
    }

    //!
    //! \brief getXResolution
    //! \return
    //!
    double getXResolution() const
    {
        return this->xResolution;
    }

    //!
    //! \brief getYResolution
    //! \return
    //!
    double getYResolution() const
    {
        return this->yResolution;
    }

    unsigned int getNodeCount() const
    {
        return xSize * ySize;
    }

    double getXLength() const
    {
        return xMax - xMin;
    }

    double getYLength() const
    {
        return yMax - yMin;
    }

    pose::CartesianPosition_2D getOriginPosition() const
    {
        return this->originPosition;
    }

protected:
    virtual void updateGridSize(const double &minX, const double &maxX, const double &minY, const double &maxY, const double &x_res, const double &y_res);

    virtual void updateGridSizeByLength(const double &x_length = 10.0, const double &y_length = 10.0,
                     const double &x_res = 0.5, const double &y_res = 0.5);

public:
    bool operator == (const BaseGridMap &rhs) const
    {
        if(this->originPosition != rhs.originPosition){
            return false;
        }
        if(this->xMin != rhs.xMin){
            return false;
        }
        if(this->xMax != rhs.xMax){
            return false;
        }
        if(this->yMin != rhs.yMin){
            return false;
        }
        if(this->yMax != rhs.yMax){
            return false;
        }
        if(this->xResolution != rhs.xResolution){
            return false;
        }
        if(this->yResolution != rhs.yResolution){
            return false;
        }
        if(this->xSize != rhs.xSize){
            return false;
        }
        if(this->ySize != rhs.ySize){
            return false;
        }
        return true;
    }

    BaseGridMap& operator = (const BaseGridMap &rhs)
    {
        this->originPosition = rhs.originPosition;
        this->xMin = rhs.xMin;
        this->xMax = rhs.xMax;
        this->yMin = rhs.yMin;
        this->yMax = rhs.yMax;
        this->xResolution = rhs.xResolution;
        this->yResolution = rhs.yResolution;
        this->xSize = rhs.xSize;
        this->ySize = rhs.ySize;
        return *this;
    }

protected:
    pose::CartesianPosition_2D originPosition; //!< Position of the map relative to the grid frame

    double xMin, yMin; //!< Description of members
    double xMax, yMax; //!< Description of members
    double xResolution, yResolution; //!< Description of members
    size_t xSize, ySize; //!< Description of members
};

} //end of namespace maps
} //end of namespace mace

#endif // BASE_GRID_MAP_H
