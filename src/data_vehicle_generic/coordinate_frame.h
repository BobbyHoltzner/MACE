#ifndef COORDINATE_FRAME_H
#define COORDINATE_FRAME_H

#include <string>
#include <stdexcept>

namespace DataVehicleGeneric
{
enum class CoordinateFrame{
    NED,
    NWU
};

inline std::string CoordinateFrameToString(const CoordinateFrame &frame) {
    switch (frame) {
    case CoordinateFrame::NED:
        return "NED";
    case CoordinateFrame::NWU:
        return "NWU";
    default:
        throw std::runtime_error("Unknown coordinate system seen");
    }
}

inline CoordinateFrame CoordinateFrameFromString(const std::string &str) {
    if(str == "NED")
        return CoordinateFrame::NED;
    if(str == "NWU")
        return CoordinateFrame::NWU;

    throw std::runtime_error("Unknown coordinate system seen");
}

}

#endif // COORDINATE_FRAME_H
