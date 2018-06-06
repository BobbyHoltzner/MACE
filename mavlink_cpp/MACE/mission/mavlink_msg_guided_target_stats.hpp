// MESSAGE GUIDED_TARGET_STATS support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief GUIDED_TARGET_STATS message
 *
 * Delete all mission items at once.
 */
struct GUIDED_TARGET_STATS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 120;
    static constexpr size_t LENGTH = 17;
    static constexpr size_t MIN_LENGTH = 17;
    static constexpr uint8_t CRC_EXTRA = 109;
    static constexpr auto NAME = "GUIDED_TARGET_STATS";


    uint8_t coordinate_frame; /*< Coordinate frame of the position vector. This field is as related to the MAV_FRAME definition. */
    float x; /*< X position of this position in the defined coordinate frame */
    float y; /*< Y position of this position in the defined coordinate frame */
    float z; /*< Z position of this position in the defined coordinate frame */
    float distance; /*< Relative distance away the system is from the target location. */


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
        ss << "  coordinate_frame: " << +coordinate_frame << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;
        ss << "  distance: " << distance << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << x;                             // offset: 0
        map << y;                             // offset: 4
        map << z;                             // offset: 8
        map << distance;                      // offset: 12
        map << coordinate_frame;              // offset: 16
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> x;                             // offset: 0
        map >> y;                             // offset: 4
        map >> z;                             // offset: 8
        map >> distance;                      // offset: 12
        map >> coordinate_frame;              // offset: 16
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
