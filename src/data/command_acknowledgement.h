#ifndef COMMAND_ACKNOWLEDGEMENT_H
#define COMMAND_ACKNOWLEDGEMENT_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class CMD_ACK{
    ACCEPTED,
    REJECTED,
    FAILED,
    UNSUPPORTED
};

inline std::string GuidedStateToString(const CMD_ACK &state) {
    switch (state) {
    case CMD_ACK::ACCEPTED:
        return "ACCEPTED";
    case CMD_ACK::REJECTED:
        return "REJECTED";
    case CMD_ACK::FAILED:
        return "FAILED";
    case CMD_ACK::UNSUPPORTED:
        return "UNSUPPORTED";
    default:
        throw std::runtime_error("Unknown cmd ack seen");
    }
}

inline CMD_ACK GuidedStateFromString(const std::string &str) {
    if(str == "ACCEPTED")
        return CMD_ACK::ACCEPTED;
    if(str == "REJECTED")
        return CMD_ACK::REJECTED;
    if(str == "FAILED")
        return CMD_ACK::FAILED;
    if(str == "UNSUPPORTED")
        return CMD_ACK::UNSUPPORTED;

    throw std::runtime_error("Unknown cmd ack seen");
}

#endif // COMMAND_ACKNOWLEDGEMENT_H
