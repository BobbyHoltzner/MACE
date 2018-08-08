#include "base_grid_map.h"

namespace mace{
namespace maps{

BaseGridMap::BaseGridMap(const double &x_length, const double &y_length,
                         const double &x_res, const double &y_res,
                         const pose::CartesianPosition_2D &position)
{
    this->originPosition = position;
    this->updateGridSizeByLength(x_length, y_length, x_res, y_res);
}

BaseGridMap::BaseGridMap(const double &x_min, const double &x_max,
                         const double &y_min, const double &y_max,
                         const double &x_res, const double &y_res,
                         const pose::CartesianPosition_2D &position)
{
    this->originPosition = position;
    this->updateGridSize(x_min,x_max,y_min,y_max,x_res,y_res);
}

BaseGridMap::BaseGridMap(const BaseGridMap &copy)
{
    this->originPosition = copy.originPosition;
    this->xMin = copy.xMin;
    this->xMax = copy.xMax;
    this->yMin = copy.yMin;
    this->yMax = copy.yMax;
    this->xResolution = copy.xResolution;
    this->yResolution = copy.yResolution;
    this->xSize = copy.xSize;
    this->ySize = copy.ySize;
}

bool BaseGridMap::updateGridSize(const double &minX, const double &maxX, const double &minY, const double &maxY, const double &x_res, const double &y_res)
{
    // Update the internal resolution memebers
    xResolution = x_res;
    yResolution = y_res;

    // Adjust sizes to adapt them to full sized cells acording to the desired resolution
    xMin = xResolution * lrint(minX / xResolution);
    xMax = xResolution * lrint(maxX / xResolution);
    yMin = yResolution * lrint(minY / yResolution);
    yMax = yResolution * lrint(maxY / yResolution);

    // Now the number of cells should be integers:
    xSize = round((xMax - xMin) / xResolution); // Ken these originally had a + 1
    ySize = round((yMax - yMin) / yResolution); // Ken these originally had a + 1
}

bool BaseGridMap::updateGridSizeByLength(const double &x_length, const double &y_length,
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
    xSize = round((xMax - xMin) / xResolution); // Ken these originally had a + 1
    ySize = round((yMax - yMin) / yResolution); // Ken these originally had a + 1
}

void BaseGridMap::setXResolution(const double &x_res)
{
    this->updateResolution(x_res,this->yResolution);
}

void BaseGridMap::setYResolution(const double &y_res)
{
    this->updateResolution(this->xResolution,y_res);
}

void BaseGridMap::updateResolution(const double &x_res, const double &y_res)
{
    this->updateGridSize(this->xMin,this->xMax,this->yMin,this->yMax,x_res,y_res);
}

void BaseGridMap::updateOriginPosition(const pose::CartesianPosition_2D &position)
{
    this->originPosition = position;
}

} //end of namespace maps
} //end of namepsace mace

