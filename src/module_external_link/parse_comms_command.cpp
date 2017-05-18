#include "module_external_link.h"

void ModuleExternalLink::ParseCommsCommand(const mace_command_long_t *message)
{
    switch(message->command)
    {
    case(MAV_CMD_GET_HOME_POSITION):
    {
        CommandItem::SpatialHome missionHome = this->getDataObject()->GetVehicleHomePostion(message->target_system);
        DataCOMMS::Mission_MACETOCOMMS missionConvert(message->target_system,message->target_component);
        mace_message_t msg = missionConvert.Home_MACETOCOMMS(missionHome);
        m_LinkMarshaler->SendMessage<mace_message_t>(m_LinkName, msg);
        break;
    }
    case(MAV_CMD_COMPONENT_ARM_DISARM):
    {
        CommandItem::ActionArm tmpArm;
        tmpArm.setTargetSystem(message->target_system);
        tmpArm.setVehicleArm(fabs(message->param1) <= 0.001 ? false : true);
        //notify core
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_ArmVehicle(this, tmpArm);
        });
        break;
    }
    case(MAV_CMD_NAV_TAKEOFF):
    {
        CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> tmpTakeoff;
        tmpTakeoff.setTargetSystem(message->target_system);
        tmpTakeoff.position.latitude = message->param5;
        tmpTakeoff.position.longitude = message->param6;
        tmpTakeoff.position.altitude = message->param7;

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_RequestVehicleTakeoff(this, tmpTakeoff);
        });
        break;
    }
    case(MAV_CMD_DO_CHANGE_SPEED):
    {
        break;
    }
    default:
        break;
    }
}
