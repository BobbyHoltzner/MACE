// MESSAGE ACK_RXMISSION support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief ACK_RXMISSION message
 *
 * This message is emitted after a mace has finished receiving a mission from another MACE instance. Seeing this response should cause MACE core to update the mission to the appropriate queue and notify other modules of the change based on the mission state received.
 */
struct ACK_RXMISSION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 102;
    static constexpr size_t LENGTH = 6;
    static constexpr size_t MIN_LENGTH = 6;
    static constexpr uint8_t CRC_EXTRA = 244;
    static constexpr auto NAME = "ACK_RXMISSION";


    uint8_t target_system; /*< System ID */
    uint8_t mission_system; /*< Mission System ID */
    uint8_t mission_creator; /*< Creator ID */
    uint8_t mission_id; /*< Mission ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */
    uint8_t mission_state; /*< The potential new mission state, see MISSION_STATE */


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
        ss << "  mission_system: " << +mission_system << std::endl;
        ss << "  mission_creator: " << +mission_creator << std::endl;
        ss << "  mission_id: " << +mission_id << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;
        ss << "  mission_state: " << +mission_state << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << target_system;                 // offset: 0
        map << mission_system;                // offset: 1
        map << mission_creator;               // offset: 2
        map << mission_id;                    // offset: 3
        map << mission_type;                  // offset: 4
        map << mission_state;                 // offset: 5
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> target_system;                 // offset: 0
        map >> mission_system;                // offset: 1
        map >> mission_creator;               // offset: 2
        map >> mission_id;                    // offset: 3
        map >> mission_type;                  // offset: 4
        map >> mission_state;                 // offset: 5
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
