// MESSAGE NEW_BOUNDARY_OBJECT support class

#pragma once

namespace mavlink {
namespace boundary {
namespace msg {

/**
 * @brief NEW_BOUNDARY_OBJECT message
 *
 * This message is emitted currently under one circumstances. A vehicle may emit this when its local planners have developed a new mission for the vehicle and have updated one of its queues. The count of the mission items in this queue is sent as its more direct method of enabling other mace instances for transmission
 */
struct NEW_BOUNDARY_OBJECT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 130;
    static constexpr size_t LENGTH = 3;
    static constexpr size_t MIN_LENGTH = 3;
    static constexpr uint8_t CRC_EXTRA = 154;
    static constexpr auto NAME = "NEW_BOUNDARY_OBJECT";


    uint8_t boundary_system; /*< System ID */
    uint8_t boundary_creator; /*< Creator ID */
    uint8_t boundary_type; /*< Boundary type, see BOUNDARY_TYPE */


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

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << boundary_system;               // offset: 0
        map << boundary_creator;              // offset: 1
        map << boundary_type;                 // offset: 2
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> boundary_system;               // offset: 0
        map >> boundary_creator;              // offset: 1
        map >> boundary_type;                 // offset: 2
    }
};

} // namespace msg
} // namespace boundary
} // namespace mavlink
