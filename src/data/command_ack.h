#ifndef COMMAND_ACK_H
#define COMMAND_ACK_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class ENUM_CommandAck{
    CA_RECEIVED=0, /* Command / mission item is ok. | */
    CA_ACCEPTED=1, /* Command / mission item is ok. | */
    CA_REJECTED=2, /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | */
    CA_NOT_SUPPORTED=3, /* Command or mission item is not supported, other commands would be accepted. | */
    CA_FAILED = 4
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
    case ENUM_CommandAck::CA_FAILED:
        return "CA_FAILED";
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
    if(str == "CA_FAILED")
        return ENUM_CommandAck::CA_FAILED;
    throw std::runtime_error("Unknown command acknowledgment seen");
}

} //end of namespace Data

#endif // COMMAND_ACK_H
