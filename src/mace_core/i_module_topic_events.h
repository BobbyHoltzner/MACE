#ifndef I_MODULE_TOPIC_EVENTS_H
#define I_MODULE_TOPIC_EVENTS_H


#include <functional>
#include "vehicle_data.h"
#include "vehicle_message.h"
#include "vehicle_object.h"
#include "topic.h"

#include "abstract_module_base.h"

namespace MaceCore
{

class IModuleTopicEvents
{
public:


    virtual void Subscribe(ModuleBase* sender, const std::string &topicName, const std::vector<int> &senderIDs = {}, const std::vector<std::string> &components = {}) = 0;

    virtual void NewTopicDataValues(const std::string &topicName, const int senderID, const TIME &time, const TopicDatagram &value) = 0;

};

} //End MaceCore Namespace


#endif // I_MODULE_TOPIC_EVENTS_H
