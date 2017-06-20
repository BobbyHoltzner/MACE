#include "command_mavlink_to_mace.h"

namespace DataMAVLINK{

Command_MAVLINKTOMACE::Command_MAVLINKTOMACE()
{

}

Data::CommandACKType Command_MAVLINKTOMACE::getMACEACKCode(const MAV_RESULT &mavACK)
{
    Data::CommandACKType rtn;
    switch (mavACK) {
    case MAV_RESULT_ACCEPTED:
    {
        rtn = Data::CommandACKType::CA_ACCEPTED;
       break;
    }
    case MAV_RESULT_TEMPORARILY_REJECTED:
    case MAV_RESULT_DENIED:
    {
        rtn = Data::CommandACKType::CA_REJECTED;
       break;
    }
    case MAV_RESULT_FAILED:
    {
        rtn = Data::CommandACKType::CA_FAILED;
       break;
    }
    case MAV_RESULT_UNSUPPORTED:
    {
        rtn = Data::CommandACKType::CA_NOT_SUPPORTED;
       break;
    }
    default:
        rtn = Data::CommandACKType::CA_UNKNOWN;
        break;
    }

    return rtn;
}

Data::CommandItemType Command_MAVLINKTOMACE::getMACECommandCode(const MAV_CMD &mavCMD)
{
    Data::CommandItemType rtn;
    switch (mavCMD) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
    {
        rtn = Data::CommandItemType::CI_ACT_ARM;
        break;
    }
    case MAV_CMD_DO_CHANGE_SPEED:
    {
        rtn = Data::CommandItemType::CI_ACT_CHANGESPEED;
        break;
    }
    case MAV_CMD_NAV_TAKEOFF:
    {
        rtn = Data::CommandItemType::CI_NAV_TAKEOFF;
        break;
    }
    case MAV_CMD_NAV_LAND:
    {
        rtn = Data::CommandItemType::CI_NAV_LAND;
       break;
    }
    case MAV_CMD_NAV_LOITER_UNLIM:
    {
        rtn = Data::CommandItemType::CI_NAV_LOITER_UNLIM;
       break;
    }
    default:
        rtn = Data::CommandItemType::CI_UNKNOWN;
        break;
    }

    return rtn;
}

} //end of namespace DataMAVLINK
