#include "action_item_topic.h"

namespace DataVehicleCommands {

const char ActionItemTopic_Name[] = "ActionItemTopic";

const MaceCore::TopicComponentStructure ActionItemTopic_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<GeneralActionItem>("actionItem");
    return structure;
}();

MaceCore::TopicDatagram ActionItemTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<GeneralActionItem>("actionItem", m_MissionItem);
    return datagram;
}

void ActionItemTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_MissionItem = datagram.GetTerminal<GeneralActionItem>("actionItem");
}

void ActionItemTopic::setActionItem(const GeneralActionItem &actionItem)
{
    m_ActionItem = actionItem;
}

} //end of namespace DataVehicleCommands
