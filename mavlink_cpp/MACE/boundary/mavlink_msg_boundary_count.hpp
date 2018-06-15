// MESSAGE BOUNDARY_COUNT support class

#pragma once

namespace mavlink {
namespace boundary {
namespace msg {

/**
 * @brief BOUNDARY_COUNT message
 *
 * This message is emitted as response to BOUNDARY_REQUEST_LIST by the agent and to initiate a write transaction. Each agent can then request the individual boundary items based on the knowledge of the total number of items.
 */
struct BOUNDARY_COUNT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 133;
    static constexpr size_t LENGTH = 5;
    static constexpr size_t MIN_LENGTH = 5;
    static constexpr uint8_t CRC_EXTRA = 168;
    static constexpr auto NAME = "BOUNDARY_COUNT";


    uint8_t boundary_system; /*< System ID */
    uint8_t boundary_creator; /*< Creator ID */
    uint8_t boundary_type; /*< Boundary type, see BOUNDARY_TYPE */
    uint16_t count; /*< Number of items defining the boundary. */


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
        ss << "  boundary_system: " << +boundary_system << std::endl;
        ss << "  boundary_creator: " << +boundary_creator << std::endl;
        ss << "  boundary_type: " << +boundary_type << std::endl;
        ss << "  count: " << count << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << count;                         // offset: 0
        map << boundary_system;               // offset: 2
        map << boundary_creator;              // offset: 3
        map << boundary_type;                 // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> count;                         // offset: 0
        map >> boundary_system;               // offset: 2
        map >> boundary_creator;              // offset: 3
        map >> boundary_type;                 // offset: 4
    }
};

} // namespace msg
} // namespace boundary
} // namespace mavlink
