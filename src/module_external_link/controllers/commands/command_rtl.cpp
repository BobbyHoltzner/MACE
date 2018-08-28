#include "command_rtl.h"

namespace ExternalLink {


    CommandRTL::CommandRTL(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan) :
        Controller_GenericShortCommand<CommandItem::SpatialRTL, (uint8_t)CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH>(cb, queue, linkChan)
    {

    }

    void CommandRTL::FillCommand(const CommandItem::SpatialRTL &commandItem, mace_command_short_t &cmd) const
    {
        UNUSED(commandItem);
        UNUSED(cmd);
    }

    void CommandRTL::BuildCommand(const mace_command_short_t &message, std::shared_ptr<CommandItem::SpatialRTL> data) const
    {
        UNUSED(message);
        UNUSED(data);
    }

}
