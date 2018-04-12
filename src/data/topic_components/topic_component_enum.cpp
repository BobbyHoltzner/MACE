#include "topic_component_enum.h"

const char TopicComponts_Enum_name[] = "enum";

const MaceCore::TopicComponentStructure TopicComponts_Enum_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("enum");
    return structure;
}();
