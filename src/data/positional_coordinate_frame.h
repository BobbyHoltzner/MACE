#ifndef POSITIONAL_COORDINATE_FRAME_H
#define POSITIONAL_COORDINATE_FRAME_H

namespace Data {

enum class PositionalFrame{
    LOCAL,
    GENERAL,
    GLOBAL
};

struct globalPos
{
    float latitude;
    float longitude;
    float altitude;
};

struct localPos
{
    double x;
    double y;
    double z;
};

}



#endif // POSITIONAL_COORDINATE_FRAME_H
