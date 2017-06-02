#ifndef COMMAND_ITEM_TOPIC_H
#define COMMAND_ITEM_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_command_item/command_item_components.h"

namespace CommandTopic{

extern const char CommandItemTopic_name[];
extern const MaceCore::TopicComponentStructure CommandItemTopic_structure;

class CommandItemTopic :public Data::NamedTopicComponentDataObject<CommandItemTopic_name, &CommandItemTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    CommandItemTopic(const std::shared_ptr<CommandItem::AbstractCommandItem> &command)
    {
        this->setCommandItem(command);
    }

    void setCommandItem(const std::shared_ptr<CommandItem::AbstractCommandItem> &command)
    {
        this->commandItem = command;
    }

    std::shared_ptr<CommandItem::AbstractCommandItem> getCommandItem() const
    {
        return commandItem;
    }

    int getOriginatingSystem() const
    {
        return commandItem->getGeneratingSystem();
    }

    int getTargetSystem() const
    {
        return commandItem->getTargetSystem();
    }

    Data::CommandItemType getCommandType() const
    {
        return commandItem->getCommandType();
    }

    Data::CoordinateFrameType getCoordinateFrame() const
    {
        return commandItem->getCoordinateFrame();
    }

private:
    std::shared_ptr<CommandItem::AbstractCommandItem> commandItem;
};

} //end of namespace CommandTopic


#endif // COMMAND_ITEM_TOPIC_H