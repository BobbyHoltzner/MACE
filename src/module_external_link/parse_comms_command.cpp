#include "module_external_link.h"

void ModuleExternalLink::ParseCommsCommand(const mavlink_command_long_t *message)
{
    switch(message->command)
    {
    case(MAV_CMD_COMPONENT_ARM_DISARM):
    {
        MissionItem::ActionArm tmpArm;
        tmpArm.setVehicleID(message->target_system);
        tmpArm.setVehicleArm(fabs(message->param1) <= 0.001 ? false : true);
        //notify core
//        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
//            ptr->RequestVehicleArm(this, tmpArm);
//        });
        break;
    }
    case(MAV_CMD_DO_CHANGE_SPEED):
    {
        break;
    }
    case(MAV_CMD_GET_HOME_POSITION):
    {
//        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
//            ptr->RequestVehicleHomePosition(this, vehicleID);
//        });
        break;
    }
    case(MAV_CMD_NAV_TAKEOFF):
    {
        MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> tmpTakeoff;
        tmpTakeoff.setVehicleID(message->target_system);
        tmpTakeoff.position.latitude = message->param5;
        tmpTakeoff.position.longitude = message->param6;
        tmpTakeoff.position.altitude = message->param7;

//        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
//            ptr->RequestVehicleTakeoff(this, tmpTakeoff);
//        });
        break;
    }
    default:
        break;
    }
}
