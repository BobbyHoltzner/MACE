// MESSAGE BOUNDARY_ITEM support class

#pragma once

namespace mavlink {
namespace boundary {
namespace msg {

/**
 * @brief BOUNDARY_ITEM message
 *
 * Message encoding a boundary item. This message is emitted to announce the presence of a boundary item and to set a boundary item on the system. The boundary item can be either in cartesian or geodetic frames with orientation defined via the frame paramter.
 */
struct BOUNDARY_ITEM : mavlink::Message {
    static constexpr msgid_t MSG_ID = 135;
    static constexpr size_t LENGTH = 18;
    static constexpr size_t MIN_LENGTH = 18;
    static constexpr uint8_t CRC_EXTRA = 12;
    static constexpr auto NAME = "BOUNDARY_ITEM";


    uint8_t boundary_system; /*< System ID */
    uint8_t boundary_creator; /*< Creator ID */
    uint8_t boundary_type; /*< Boundary type, see BOUNDARY_TYPE */
    uint16_t seq; /*< Sequence */
    uint8_t frame; /*< The coordinate system of the boundary. see MAV_FRAME in mavlink_types.h */
    float x; /*< PARAM5 / local: x position, global: latitude */
    float y; /*< PARAM6 / y position: global: longitude */
    float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame. */


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
        ss << "  seq: " << seq << std::endl;
        ss << "  frame: " << +frame << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << x;                             // offset: 0
        map << y;                             // offset: 4
        map << z;                             // offset: 8
        map << seq;                           // offset: 12
        map << boundary_system;               // offset: 14
        map << boundary_creator;              // offset: 15
        map << boundary_type;                 // offset: 16
        map << frame;                         // offset: 17
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> x;                             // offset: 0
        map >> y;                             // offset: 4
        map >> z;                             // offset: 8
        map >> seq;                           // offset: 12
        map >> boundary_system;               // offset: 14
        map >> boundary_creator;              // offset: 15
        map >> boundary_type;                 // offset: 16
        map >> frame;                         // offset: 17
    }
};

} // namespace msg
} // namespace boundary
} // namespace mavlink
