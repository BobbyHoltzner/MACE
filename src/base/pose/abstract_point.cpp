#include "base_point.h"

using namespace base;
using namespace pose;

AbstractPoint::AbstractPoint()
{
    this->m_CoordinateFrame = CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

AbstractPoint::AbstractPoint(const AbstractPoint &copy)
{
    this->m_CoordinateFrame = copy.getCoordinateFrame();
}

AbstractPoint::AbstractPoint(const CoordinateFrameType &coordinateFrame)
{
    this->m_CoordinateFrame = coordinateFrame;
}
