#include "dynamic_2D_grid.h"

namespace Maps {

template <class T>
Dynamic2DGrid<T>::Dynamic2DGrid(const double &x_min, const double &x_max,
                             const double &y_min, const double &y_max,
                             const double &x_res, const double &y_res,
                             const T *fill_value):
    m_dataMap(), xMin(0.0), yMin(0.0), xMax(0.0), yMax(0.0), xResolution(0.0), yResolution(0.0),
    xSize(0), ySize(0)

{
    setGridSize(x_min,x_max,y_min,y_max,x_res,y_res, fill_value);
}

template <class T>
void Dynamic2DGrid<T>::setGridSize(const double &x_min, const double &x_max, const double &y_min, const double &y_max, const double &x_res, const double &y_res, const T *fill_value)
{
    // Update the internal resolution memebers
    xResolution = x_res;
    yResolution = y_res;

    // Adjust sizes to adapt them to full sized cells acording to the desired resolution
    xMin = xResolution * lrint(x_min / xResolution);
    xMax = xResolution * lrint(x_max / xResolution);
    yMin = yResolution * lrint(y_min / yResolution);
    yMax = yResolution * lrint(y_max / yResolution);

    // Now the number of cells should be integers:
    xSize = round((xMax - yMin) / xResolution);
    ySize = round((yMax - yMin) / yResolution);

    // Cells memory:
    if (fill_value)
        m_dataMap.assign(xSize * ySize, *fill_value);
    else
        m_dataMap.resize(xSize * ySize);
}

} //end of namespace Maps
