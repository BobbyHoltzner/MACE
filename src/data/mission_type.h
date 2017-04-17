#ifndef MISSION_TYPE_H
#define MISSION_TYPE_H

#include <stdint.h>
#include <string.h>

namespace Data
{

enum class MissionTypeState
{
    PROPOSED,
    CURRENT
};

enum class MissionType : uint8_t
{
    AUTO_CURRENT,
    AUTO_PROPOSED,
    GUIDED_CURRENT,
    GUIDED_PROPOSED
};


//inline MACE_MISSION_TYPE MissionTypeToComms(const MissionType &state) {
//    switch (state) {
//    case MissionType::AUTO_PROPOSED:
//        return MACE_MISSION_TYPE_AUTO_PROPOSED;
//    case MissionType::AUTO_CURRENT:
//        return MACE_MISSION_TYPE_AUTO;
//    case MissionType::GUIDED_PROPOSED:
//        return MACE_MISSION_TYPE_GUIDED_PROPOSED;
//    case MissionType::GUIDED_CURRENT:
//        return MACE_MISSION_TYPE_GUIDED;
//    default:
//        throw std::runtime_error("Unknown MissionType seen");
//    }
//}

//inline MissionType MissionTypeFromComms(const MACE_MISSION_TYPE &state) {
//    if(state == MACE_MISSION_TYPE_AUTO_PROPOSED)
//        return MissionType::AUTO_PROPOSED;
//    if(state == MACE_MISSION_TYPE_AUTO)
//        return MissionType::AUTO_CURRENT;
//    if(state == MACE_MISSION_TYPE_GUIDED_PROPOSED)
//        return MissionType::GUIDED_PROPOSED;
//    if(state == MACE_MISSION_TYPE_GUIDED)
//        return MissionType::GUIDED_CURRENT;

//    throw std::runtime_error("Unknown MACE_MISSION_TYPE seen");
//}

inline std::string MissionTypeToString(const MissionType &state) {
    switch (state) {
    case MissionType::AUTO_PROPOSED:
        return "AUTO_PROPOSED";
    case MissionType::AUTO_CURRENT:
        return "AUTO_CURRENT";
    case MissionType::GUIDED_PROPOSED:
        return "GUIDED_PROPOSED";
    case MissionType::GUIDED_CURRENT:
        return "GUIDED_CURRENT";
    default:
        throw std::runtime_error("Unknown MissionType seen");
    }
}

inline MissionType MissionTypeFromString(const std::string &str) {
    if(str == "AUTO_PROPOSED")
        return MissionType::AUTO_PROPOSED;
    if(str == "AUTO_CURRENT")
        return MissionType::AUTO_CURRENT;
    if(str == "GUIDED_PROPOSED")
        return MissionType::GUIDED_PROPOSED;
    if(str == "GUIDED_CURRENT")
        return MissionType::GUIDED_CURRENT;

    throw std::runtime_error("Unknown std::string seen");
}

}

#endif // MISSION_TYPE_H
