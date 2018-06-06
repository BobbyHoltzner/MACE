// MESSAGE MISSION_CURRENT support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief MISSION_CURRENT message
 *
 * Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
 */
struct MISSION_CURRENT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 111;
    static constexpr size_t LENGTH = 6;
    static constexpr size_t MIN_LENGTH = 6;
    static constexpr uint8_t CRC_EXTRA = 233;
    static constexpr auto NAME = "MISSION_CURRENT";


    uint8_t mission_system; /*< Mission System ID */
    uint8_t mission_creator; /*< Creator ID */
    uint8_t mission_id; /*< Mission ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */
    uint16_t seq; /*< Sequence */


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
        ss << "  mission_system: " << +mission_system << std::endl;
        ss << "  mission_creator: " << +mission_creator << std::endl;
        ss << "  mission_id: " << +mission_id << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;
        ss << "  seq: " << seq << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << seq;                           // offset: 0
        map << mission_system;                // offset: 2
        map << mission_creator;               // offset: 3
        map << mission_id;                    // offset: 4
        map << mission_type;                  // offset: 5
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> seq;                           // offset: 0
        map >> mission_system;                // offset: 2
        map >> mission_creator;               // offset: 3
        map >> mission_id;                    // offset: 4
        map >> mission_type;                  // offset: 5
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
