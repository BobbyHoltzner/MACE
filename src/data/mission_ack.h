#ifndef MISSION_RESULT_H
#define MISSION_RESULT_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class MISSION_RESULT{
    MISSION_RESULT_ACCEPTED=0, /* mission accepted OK | */
    MISSION_RESULT_ERROR=1, /* generic error / not accepting mission commands at all right now | */
    MISSION_RESULT_UNSUPPORTED_FRAME=2, /* coordinate frame is not supported | */
    MISSION_RESULT_UNSUPPORTED=3, /* command is not supported | */
    MISSION_RESULT_NO_SPACE=4, /* mission item exceeds storage space | */
    MISSION_RESULT_INVALID=5, /* one of the parameters has an invalid value | */
    MISSION_RESULT_INVALID_SEQUENCE=13, /* received waypoint out of sequence | */
    MISSION_RESULT_DENIED=14, /* not accepting any mission commands from this communication partner | */
    MISSION_RESULT_DOES_NOT_EXIST=15, /* the requested mission with the associated key does not exist. | */
    MISSION_RESULT_RESULT_ENUM_END=16
};

} //end of namespace Data

#endif // MISSION_RESULT_H
