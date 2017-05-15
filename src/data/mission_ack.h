#ifndef MISSION_ACK_H
#define MISSION_ACK_H
namespace Data
{

enum class MissionACKType{
    MA_RECEIVED=0, /* Command / mission item is ok. | */
    MA_ACCEPTED=1, /* Command / mission item is ok. | */
    MA_REJECTED=2, /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | */
    MA_NOT_SUPPORTED=3, /* Command or mission item is not supported, other commands would be accepted. | */
    MA_COORDINATE_FRAME_NOT_SUPPORTED=4, /* The coordinate frame of this command / mission item is not supported. | */
    MA_COORDINATES_OUT_OF_RANGE=5, /* The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible. | */
    MA_X_LAT_OUT_OF_RANGE=6, /* The X or latitude value is out of range. | */
    MA_Y_LON_OUT_OF_RANGE=7, /* The Y or longitude value is out of range. | */
    MA_Z_ALT_OUT_OF_RANGE=8 /* The Z or altitude value is out of range. | */
};

inline std::string CommandACKToString(const MissionACKType &frame) {
    switch (frame) {
    case MissionACKType::MA_RECEIVED:
        return "MA_RECEIVED";
    case MissionACKType::MA_ACCEPTED:
        return "MA_ACCEPTED";
    case MissionACKType::MA_REJECTED:
        return "MA_REJECTED";
    case MissionACKType::MA_NOT_SUPPORTED:
        return "MA_NOT_SUPPORTED";
    case MissionACKType::MA_COORDINATE_FRAME_NOT_SUPPORTED:
        return "MA_COORDINATE_FRAME_NOT_SUPPORTED";
    case MissionACKType::MA_COORDINATES_OUT_OF_RANGE:
        return "MA_COORDINATES_OUT_OF_RANGE";
    case MissionACKType::MA_X_LAT_OUT_OF_RANGE:
        return "MA_X_LAT_OUT_OF_RANGE";
    case MissionACKType::MA_Y_LON_OUT_OF_RANGE:
        return "MA_Y_LON_OUT_OF_RANGE";
    case MissionACKType::MA_Z_ALT_OUT_OF_RANGE:
        return "MA_Z_ALT_OUT_OF_RANGE";
    default:
        throw std::runtime_error("Unknown mission acknowledgement seen");
    }
}

inline MissionACKType CommandACKFromString(const std::string &str) {
    if(str == "MA_RECEIVED")
        return MissionACKType::MA_RECEIVED;
    if(str == "MA_ACCEPTED")
        return MissionACKType::MA_ACCEPTED;
    if(str == "MA_REJECTED")
        return MissionACKType::MA_REJECTED;
    if(str == "MA_NOT_SUPPORTED")
        return MissionACKType::MA_NOT_SUPPORTED;
    if(str == "MA_COORDINATE_FRAME_NOT_SUPPORTED")
        return MissionACKType::MA_COORDINATE_FRAME_NOT_SUPPORTED;
    if(str == "MA_COORDINATES_OUT_OF_RANGE")
        return MissionACKType::MA_COORDINATES_OUT_OF_RANGE;
    if(str == "MA_X_LAT_OUT_OF_RANGE")
        return MissionACKType::MA_X_LAT_OUT_OF_RANGE;
    if(str == "MA_Y_LON_OUT_OF_RANGE")
        return MissionACKType::MA_Y_LON_OUT_OF_RANGE;
    if(str == "MA_Z_ALT_OUT_OF_RANGE")
        return MissionACKType::MA_Z_ALT_OUT_OF_RANGE;
    throw std::runtime_error("Unknown mission acknowledgment seen");
}

} //end of namespace Data

#endif // MISSION_ACK_H
