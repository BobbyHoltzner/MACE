// MESSAGE HOME_POSITION_ACK support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief HOME_POSITION_ACK message
 *
 * The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
 */
struct HOME_POSITION_ACK : mavlink::Message {
    static constexpr msgid_t MSG_ID = 119;
    static constexpr size_t LENGTH = 2;
    static constexpr size_t MIN_LENGTH = 2;
    static constexpr uint8_t CRC_EXTRA = 21;
    static constexpr auto NAME = "HOME_POSITION_ACK";


    uint8_t target_system; /*< System ID. */
    uint8_t ack; /*< Acknowledgement of the home position set request. */


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
        ss << "  ack: " << +ack << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << target_system;                 // offset: 0
        map << ack;                           // offset: 1
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> target_system;                 // offset: 0
        map >> ack;                           // offset: 1
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
