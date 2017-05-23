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
            ptr->Event_IssueCommandSystemArm(this, tmpArm);
        });
        break;
    }
    case(MAV_CMD_NAV_TAKEOFF):
    {
        CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> tmpTakeoff;
        tmpTakeoff.setTargetSystem(message->target_system);
        tmpTakeoff.setPositionFlag((message->param1 > 0.0) ? true : false);
        tmpTakeoff.position.latitude = message->param5;
        tmpTakeoff.position.longitude = message->param6;
        tmpTakeoff.position.altitude = message->param7;

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_IssueCommandTakeoff(this, tmpTakeoff);
        });
        break;
    }
    case(MAV_CMD_NAV_LAND):
    {
        CommandItem::SpatialLand<DataState::StateGlobalPosition> tmpLand;
        tmpLand.setTargetSystem(message->target_system);
        tmpLand.setLandFlag((message->param1 > 0.0) ? true : false);
        tmpLand.position.latitude = message->param5;
        tmpLand.position.longitude = message->param6;
        tmpLand.position.altitude = message->param7;
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_IssueCommandLand(this, tmpLand);
        });
        break;
    }
    case(MAV_CMD_NAV_RETURN_TO_LAUNCH):
    {
        CommandItem::SpatialRTL tmpRTL;
        tmpRTL.setTargetSystem(message->target_system);
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_IssueCommandRTL(this, tmpRTL);
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
