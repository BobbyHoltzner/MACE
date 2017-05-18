#ifndef MISSION_HOME_TOPIC_H
#define MISSION_HOME_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_command_item/spatial_items/spatial_home.h"

namespace MissionTopic{

extern const char MissionHomeTopic_name[];
extern const MaceCore::TopicComponentStructure MissionHomeTopic_structure;

class MissionHomeTopic :public Data::NamedTopicComponentDataObject<MissionHomeTopic_name, &MissionHomeTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionHomeTopic()
    {

    }

    MissionHomeTopic(const std::shared_ptr<CommandItem::AbstractCommandItem> &obj)
    {
        this->setHome(obj);
    }


    void setHome(const std::shared_ptr<CommandItem::AbstractCommandItem> &homeItem){
        item = homeItem;
    }

    std::shared_ptr<CommandItem::AbstractCommandItem> getHome(){
        return item;
    }


private:
    std::shared_ptr<CommandItem::AbstractCommandItem> item;
};

} //end of namespace MissionTopic

#endif // MISSION_HOME_TOPIC_H
