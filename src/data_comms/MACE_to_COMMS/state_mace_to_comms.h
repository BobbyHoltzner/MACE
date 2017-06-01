#ifndef STATE_MACE_TO_COMMS_H
#define STATE_MACE_TO_COMMS_H

#include <memory>

#include "mace.h"
#include "common/common.h"

#include "data/coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

namespace DataCOMMS{

class State_MACETOCOMMS
{
public:

    static mace_message_t AttitudeStateFull_MACETOCOMMS(const DataState::StateAttitude &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);
    static mace_message_t AttitudeStateFullTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

    static mace_message_t AttitudeState_MACETOCOMMS(const DataState::StateAttitude &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);
    static mace_message_t AttitudeStateTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

    static mace_message_t AttitudeRates_MACETOCOMMS(const DataState::StateAttitude &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);
    static mace_message_t AttitudeRatesTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

    static mace_message_t GlobalPosition_MACETOCOMMS(const DataState::StateGlobalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);
    static mace_message_t GlobalPositionTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

    static mace_message_t LocalPosition_MACETOCOMMS(const DataState::StateLocalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);
    static mace_message_t LocalPositionTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateLocalPositionTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

};

} //end of namespace DataCOMMS

#endif // STATE_MACE_TO_COMMS_H
