#include "base_grid_map.h"

namespace mace{
namespace maps{

void BaseGridMap::updateGridSize(const double &x_length, const double &y_length,
                                 const double &x_res, const double &y_res)
{
    // Update the internal resolution memebers
    xResolution = x_res;
    yResolution = y_res;

    // Adjust sizes to adapt them to full sized cells acording to the desired resolution
    xMin = -(lrint(x_length / xResolution) / 2) + position.getXPosition();
    xMax = (lrint(x_length / xResolution) / 2) + position.getXPosition();
    yMin = (lrint(y_length / yResolution) / 2) + position.getYPosition();
    yMax = (lrint(y_length / yResolution) / 2) + position.getYPosition();

    // Now the number of cells should be integers:
    xSize = round((xMax - xMin) / xResolution);
    ySize = round((yMax - yMin) / yResolution);
}

void BaseGridMap::updatePosition(const pose::CartesianPosition_2D &position)
{
    this->position = position;
}

} //end of namespace maps
} //end of namepsace mace

