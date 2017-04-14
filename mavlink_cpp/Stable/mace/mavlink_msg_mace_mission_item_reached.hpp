// MESSAGE MACE_MISSION_ITEM_REACHED support class

#pragma once

namespace mavlink {
namespace MACE {
namespace msg {

/**
 * @brief MACE_MISSION_ITEM_REACHED message
 *
 * A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next MISSION.
 */
struct MACE_MISSION_ITEM_REACHED : mavlink::Message {
    static constexpr msgid_t MSG_ID = 309;
    static constexpr size_t LENGTH = 3;
    static constexpr size_t MIN_LENGTH = 3;
    static constexpr uint8_t CRC_EXTRA = 190;
    static constexpr auto NAME = "MACE_MISSION_ITEM_REACHED";


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
        ss << "  seq: " << seq << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << seq;                           // offset: 0
        map << mission_type;                  // offset: 2
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> seq;                           // offset: 0
        map >> mission_type;                  // offset: 2
    }
};

} // namespace msg
} // namespace MACE
} // namespace mavlink
