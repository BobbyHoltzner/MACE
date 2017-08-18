#ifndef DYNAMIC_2D_GRID_H
#define DYNAMIC_2D_GRID_H

#include <vector>
#include <math.h>

namespace Maps
{

template <class T>
class Dynamic2DGrid
{
public:
    Dynamic2DGrid(const double &x_min = -10.0, const double &x_max = 10.0,
                  const double &y_min = -10.0, const double &y_max = 10.0,
                  const double &x_res = 0.25, const double &y_res = 0.25,
                  const T *fill_value = nullptr);

    virtual ~Dynamic2DGrid();

    void setGridSize(const double &x_min, const double &x_max,
                     const double &y_min, const double &y_max,
                     const double &x_res, const double &y_res,
                     const T *fill_value = nullptr);

    void clear();

    void fill(const T& value);

    virtual void resize(const double &x_min, const double &x_max,
                     const double &y_min, const double &y_max,
                     const double &x_res, const double &y_res,
                     const T &fill_value, const double &margian = 2.0);

    T* getCellByPos(const double &x, const double &y) const;

    T* getCellByIndex(const unsigned int &xIndex, const unsigned int &yIndex);

    int indexFromXPos(const double &x) const
    {
        return static_cast<int>((x - xMin) / xResolution);
    }

    int indexFromYPos(const double &y) const
    {
        return static_cast<int>((y - yMin) / yResolution);
    }

    int indexFromPos(const double &x, const double &y) const
    {
        return indexFromXPos(x) + indexFromYPos(y) * this->xSize;
    }

    size_t getSizeX() const
    {
        return this->xSize;
    }

    size_t getSizeY() const
    {
        return this->ySize;
    }

    double getXMin() const
    {
        return this->xMin;
    }
    double getYMin() const
    {
        return this->yMin;
    }
    double getXMax() const
    {
        return this->xMax;
    }
    double getYMax() const
    {
        return this->yMax;
    }
    double getXResolution() const
    {
        return this->xResolution;
    }
    double getYResolution() const
    {
        return this->yResolution;
    }


protected:
    std::vector<T> m_dataMap;

    double xMin, yMin;
    double xMax, yMax;
    double xResolution, yResolution;
    size_t xSize, ySize;

};

} //end of namespace Maps
#endif // DYNAMIC_2D_GRID_H
