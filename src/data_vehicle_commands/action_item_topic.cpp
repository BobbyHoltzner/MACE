#include "action_item_topic.h"

namespace DataVehicleCommands {

const char ActionItemTopic_Name[] = "ActionItemTopic";

const MaceCore::TopicComponentStructure ActionItemTopic_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<ActionItemTypes>("actionItemType");
    structure.AddTerminal<std::shared_ptr<GeneralActionItem>>("actionItem");
    return structure;
}();

MaceCore::TopicDatagram ActionItemTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<ActionItemTypes>("actionItemType", m_ActionItemType);
    datagram.AddTerminal<std::shared_ptr<GeneralActionItem>>("actionItem", m_ActionItem);
    return datagram;
}

void ActionItemTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_ActionItemType = datagram.GetTerminal<ActionItemTypes>("actionItemType");
    m_ActionItem = datagram.GetTerminal<std::shared_ptr<GeneralActionItem>>("actionItem");
}

void ActionItemTopic::setActionItem(const std::shared_ptr<GeneralActionItem> &actionItem)
{
    m_ActionItemType = actionItem->getActionItemType();
    m_ActionItem = actionItem;
}

std::shared_ptr<GeneralActionItem> ActionItemTopic::getActionItem() const
{
    return m_ActionItem;
}

ActionItemTypes ActionItemTopic::getActionItemType() const
{
    return m_ActionItemType;
}


} //end of namespace DataVehicleCommands
