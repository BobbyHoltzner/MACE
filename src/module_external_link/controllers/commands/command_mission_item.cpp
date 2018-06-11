#include "command_mission_item.h"

namespace ExternalLink {

    CommandMissionItem::CommandMissionItem(const Controllers::IMessageNotifier<mace_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan) :
        Controller_GenericShortCommand<CommandItem::ActionMissionCommand, (uint8_t)CommandItem::COMMANDITEM::CI_ACT_MISSIONCOMMAND>(cb, queue, linkChan)
    {

    }

    void CommandMissionItem::FillCommand(const CommandItem::ActionMissionCommand &commandItem, mace_command_short_t &cmd) const
    {
        cmd.param = (uint8_t)commandItem.getMissionCommandAction();
    }

    void CommandMissionItem::BuildCommand(const mace_command_short_t &message, std::shared_ptr<CommandItem::ActionMissionCommand> data) const
    {
        data->setMissionCommandType((Data::MissionCommandAction)message.param);
    }

}
