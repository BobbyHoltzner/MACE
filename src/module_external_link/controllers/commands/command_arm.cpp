#include "command_arm.h"

namespace ExternalLink {

    CommandARM::CommandARM(const Controllers::IMessageNotifier<mace_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan) :
        Controller_GenericShortCommand<CommandItem::ActionArm, (uint8_t)CommandItem::COMMANDITEM::CI_ACT_ARM>(cb, queue, linkChan)
    {

    }

    void CommandARM::FillCommand(const CommandItem::ActionArm &commandItem, mace_command_short_t &cmd) const
    {
        cmd.param = commandItem.getRequestArm();
    }

    void CommandARM::BuildCommand(const mace_command_short_t &message, std::shared_ptr<CommandItem::ActionArm> data) const
    {
        data->setVehicleArm(message.param);
    }

}
