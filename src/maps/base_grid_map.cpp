#include "base_grid_map.h"

namespace mace{
namespace maps{

BaseGridMap::BaseGridMap(const double &x_length, const double &y_length,
                         const double &x_res, const double &y_res,
                         const pose::CartesianPosition_2D &position)
{
    this->originPosition = position;
    this->updateGridSizeEven(x_length, y_length, x_res, y_res);
}

BaseGridMap::BaseGridMap(const double &x_min, const double &x_max,
                         const double &y_min, const double &y_max,
                         const double &x_res, const double &y_res,
                         const pose::CartesianPosition_2D &origin)
{
    this->originPosition = origin;
    this->updateGridSize(x_min, x_max, y_min, y_max, x_res, y_res);
}

void BaseGridMap::updateGridSize(const double &x_min, const double &x_max,
                                 const double &y_min, const double &y_max,
                                 const double &x_res, const double &y_res)
{
    // Update the internal resolution memebers
    xResolution = x_res;
    yResolution = y_res;

    // Adjust sizes to adapt them to full sized cells acording to the desired resolution
    xMin = x_min;
    xMax = x_max;
    yMin = y_min;
    yMax = y_max;

    // Now the number of cells should be integers:
    xSize = round((xMax - xMin) / xResolution) + 1;
    ySize = round((yMax - yMin) / yResolution) + 1;
}

void BaseGridMap::updateGridSizeEven(const double &x_length, const double &y_length,
                                 const double &x_res, const double &y_res)
{
    // Update the internal resolution memebers
    xResolution = x_res;
    yResolution = y_res;

    // Adjust sizes to adapt them to full sized cells acording to the desired resolution
    xMin = -(xResolution * lrint((x_length / 2) / xResolution)) + originPosition.getXPosition();
    xMax = (xResolution * lrint((x_length / 2) / xResolution)) + originPosition.getXPosition();
    yMin = -(yResolution * lrint((y_length / 2) / yResolution)) + originPosition.getYPosition();
    yMax = (yResolution * lrint((y_length / 2) / yResolution)) + originPosition.getYPosition();

    // Now the number of cells should be integers:
    xSize = round((xMax - xMin) / xResolution) + 1;
    ySize = round((yMax - yMin) / yResolution) + 1;
}

void BaseGridMap::updatePosition(const pose::CartesianPosition_2D &position)
{
    this->originPosition = position;
    updateGridSizeEven(xMax-xMin,yMax-yMin,xResolution,yResolution);
}

} //end of namespace maps
} //end of namepsace mace

