#ifndef COMMAND_ACK_H
#define COMMAND_ACK_H

namespace Data
{
enum class ENUM_CommandAck{
    CA_RECEIVED=0, /* Command / mission item is ok. | */
    CA_ACCEPTED=1, /* Command / mission item is ok. | */
    CA_REJECTED=2, /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | */
    CA_NOT_SUPPORTED=3, /* Command or mission item is not supported, other commands would be accepted. | */
    CA_COORDINATE_FRAME_NOT_SUPPORTED=4, /* The coordinate frame of this command / mission item is not supported. | */
    CA_COORDINATES_OUT_OF_RANGE=5, /* The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible. | */
    CA_X_LAT_OUT_OF_RANGE=6, /* The X or latitude value is out of range. | */
    CA_Y_LON_OUT_OF_RANGE=7, /* The Y or longitude value is out of range. | */
    CA_Z_ALT_OUT_OF_RANGE=8 /* The Z or altitude value is out of range. | */
};

inline std::string CommandACKToString(const ENUM_CommandAck &frame) {
    switch (frame) {
    case ENUM_CommandAck::CA_RECEIVED:
        return "CA_RECEIVED";
    case ENUM_CommandAck::CA_ACCEPTED:
        return "CA_ACCEPTED";
    case ENUM_CommandAck::CA_REJECTED:
        return "CA_REJECTED";
    case ENUM_CommandAck::CA_NOT_SUPPORTED:
        return "CA_NOT_SUPPORTED";
    case ENUM_CommandAck::CA_COORDINATE_FRAME_NOT_SUPPORTED:
        return "CA_COORDINATE_FRAME_NOT_SUPPORTED";
    case ENUM_CommandAck::CA_COORDINATES_OUT_OF_RANGE:
        return "CA_COORDINATES_OUT_OF_RANGE";
    case ENUM_CommandAck::CA_X_LAT_OUT_OF_RANGE:
        return "CA_X_LAT_OUT_OF_RANGE";
    case ENUM_CommandAck::CA_Y_LON_OUT_OF_RANGE:
        return "CA_Y_LON_OUT_OF_RANGE";
    case ENUM_CommandAck::CA_Z_ALT_OUT_OF_RANGE:
        return "CA_Z_ALT_OUT_OF_RANGE";
    default:
        throw std::runtime_error("Unknown command acknowledgement seen");
    }
}

inline ENUM_CommandAck CommandACKFromString(const std::string &str) {
    if(str == "CA_RECEIVED")
        return ENUM_CommandAck::CA_RECEIVED;
    if(str == "CA_ACCEPTED")
        return ENUM_CommandAck::CA_ACCEPTED;
    if(str == "CA_REJECTED")
        return ENUM_CommandAck::CA_REJECTED;
    if(str == "CA_NOT_SUPPORTED")
        return ENUM_CommandAck::CA_NOT_SUPPORTED;
    if(str == "CA_COORDINATE_FRAME_NOT_SUPPORTED")
        return ENUM_CommandAck::CA_COORDINATE_FRAME_NOT_SUPPORTED;
    if(str == "CA_COORDINATES_OUT_OF_RANGE")
        return ENUM_CommandAck::CA_COORDINATES_OUT_OF_RANGE;
    if(str == "CA_X_LAT_OUT_OF_RANGE")
        return ENUM_CommandAck::CA_X_LAT_OUT_OF_RANGE;
    if(str == "CA_Y_LON_OUT_OF_RANGE")
        return ENUM_CommandAck::CA_Y_LON_OUT_OF_RANGE;
    if(str == "CA_Z_ALT_OUT_OF_RANGE")
        return ENUM_CommandAck::CA_Z_ALT_OUT_OF_RANGE;
    throw std::runtime_error("Unknown command acknowledgment seen");
}

} //end of namespace Data

#endif // COMMAND_ACK_H
