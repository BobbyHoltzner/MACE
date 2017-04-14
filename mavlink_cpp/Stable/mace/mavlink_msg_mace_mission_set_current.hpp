// MESSAGE MACE_MISSION_SET_CURRENT support class

#pragma once

namespace mavlink {
namespace MACE {
namespace msg {

/**
 * @brief MACE_MISSION_SET_CURRENT message
 *
 * Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
 */
struct MACE_MISSION_SET_CURRENT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 304;
    static constexpr size_t LENGTH = 5;
    static constexpr size_t MIN_LENGTH = 5;
    static constexpr uint8_t CRC_EXTRA = 83;
    static constexpr auto NAME = "MACE_MISSION_SET_CURRENT";


    uint8_t target_system; /*< System ID */
    uint8_t target_component; /*< Component ID */
    uint16_t seq; /*< Sequence */
    uint8_t mission_type; /*< Mission type, see MACE_MISSION_PROFILE */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  target_component: " << +target_component << std::endl;
        ss << "  seq: " << seq << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << seq;                           // offset: 0
        map << target_system;                 // offset: 2
        map << target_component;              // offset: 3
        map << mission_type;                  // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> seq;                           // offset: 0
        map >> target_system;                 // offset: 2
        map >> target_component;              // offset: 3
        map >> mission_type;                  // offset: 4
    }
};

} // namespace msg
} // namespace MACE
} // namespace mavlink
