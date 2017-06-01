#include "state_mace_to_comms.h"

namespace DataCOMMS {


mace_message_t State_MACETOCOMMS::AttitudeStateFullTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataState::StateAttitude newAttitude = *topicItem.get();
    mace_message_t msg = AttitudeStateFull_MACETOCOMMS(newAttitude,systemID,compID,chan);
    return(msg);
}

mace_message_t State_MACETOCOMMS::AttitudeStateFull_MACETOCOMMS(const DataState::StateAttitude &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    mace_message_t msg;
    mace_attitude_state_full_t attitude;
    attitude.roll = stateItem.roll;
    attitude.pitch = stateItem.pitch;
    attitude.yaw = stateItem.yaw;
    attitude.rollspeed = stateItem.rollRate;
    attitude.pitchspeed = stateItem.pitchRate;
    attitude.yawspeed = stateItem.yawRate;
    mace_msg_attitude_state_full_encode_chan(systemID,compID,chan,&msg,&attitude);
    return(msg);
}

mace_message_t State_MACETOCOMMS::AttitudeStateTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataState::StateAttitude newAttitude = *topicItem.get();
    mace_message_t msg = AttitudeState_MACETOCOMMS(newAttitude,systemID,compID,chan);
    return(msg);
}

mace_message_t State_MACETOCOMMS::AttitudeState_MACETOCOMMS(const DataState::StateAttitude &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    mace_message_t msg;
    mace_attitude_t attitude;
    attitude.roll = stateItem.roll;
    attitude.pitch = stateItem.pitch;
    attitude.yaw = stateItem.yaw;
    mace_msg_attitude_encode_chan(systemID,compID,chan,&msg,&attitude);
    return(msg);
}

mace_message_t State_MACETOCOMMS::AttitudeRatesTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataState::StateAttitude newAttitude = *topicItem.get();
    mace_message_t msg = AttitudeRates_MACETOCOMMS(newAttitude,systemID,compID,chan);
    return(msg);
}

mace_message_t State_MACETOCOMMS::AttitudeRates_MACETOCOMMS(const DataState::StateAttitude &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    mace_message_t msg;
    mace_attitude_rates_t attitude;
    attitude.rollspeed = stateItem.rollRate;
    attitude.pitchspeed = stateItem.pitchRate;
    attitude.yawspeed = stateItem.yawRate;
    mace_msg_attitude_rates_encode_chan(systemID,compID,chan,&msg,&attitude);
    return(msg);
}

mace_message_t State_MACETOCOMMS::GlobalPositionTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataState::StateGlobalPosition newGlobalPosition = *topicItem.get();
    mace_message_t msg = GlobalPosition_MACETOCOMMS(newGlobalPosition,systemID,compID,chan);
    return(msg);
}

mace_message_t State_MACETOCOMMS::GlobalPosition_MACETOCOMMS(const DataState::StateGlobalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    mace_message_t msg;
    mace_global_position_int_t position;
    position.lat = (int32_t)(stateItem.latitude * pow(10,7));
    position.lon = (int32_t)(stateItem.longitude * pow(10,7));
    position.alt = (int32_t)(stateItem.altitude * 1000.0);
    mace_msg_global_position_int_encode_chan(systemID,compID,chan,&msg,&position);
    return(msg);
}

mace_message_t State_MACETOCOMMS::LocalPositionTopicPTR_MACETOCOMMS(const std::shared_ptr<DataStateTopic::StateLocalPositionTopic> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataState::StateLocalPosition newLocalPosition = *topicItem.get();
    mace_message_t msg = LocalPosition_MACETOCOMMS(newLocalPosition,systemID,compID,chan);
    return(msg);
}

mace_message_t State_MACETOCOMMS::LocalPosition_MACETOCOMMS(const DataState::StateLocalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    mace_message_t msg;
    mace_local_position_ned_t position;
    position.x = stateItem.x;
    position.y = stateItem.y;
    position.z = stateItem.z;
    mace_msg_local_position_ned_encode_chan(systemID,compID,chan,&msg,&position);
    return(msg);
}

} //end of namespace DataCOMMS
