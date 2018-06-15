#include "command_land.h"

namespace ExternalLink {

    CommandLand::CommandLand(const Controllers::IMessageNotifier<mace_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan) :
        Controller_GenericLongCommand<CommandItem::SpatialLand, (uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND>(cb, queue, linkChan)
    {

    }


    void CommandLand::FillCommand(const CommandItem::SpatialLand &commandItem, mace_command_long_t &cmd) const
    {
        if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
        {
            cmd.param2 = commandItem.position->getPosXFlag();
            cmd.param3 = commandItem.position->getPosYFlag();
            cmd.param4 = commandItem.position->getPosZFlag();
            cmd.param5 = commandItem.position->getX() * pow(10,7);
            cmd.param6 = commandItem.position->getY() * pow(10,7);
            cmd.param7 = commandItem.position->getZ() * 1000;
        }
    }

    void CommandLand::BuildCommand(const mace_command_long_t &message, std::shared_ptr<CommandItem::SpatialLand> data) const
    {
        data->setTargetSystem(message.target_system);

        if(message.param2 == 1)
        {
            data->position->setX(message.param5);
        }
        if(message.param3 == 1)
        {
            data->position->setY(message.param6);
        }
        if(message.param4 == 1)
        {
            data->position->setZ(message.param7);
        }
    }

}
