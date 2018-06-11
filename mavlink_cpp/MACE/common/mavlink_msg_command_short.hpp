// MESSAGE COMMAND_SHORT support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief COMMAND_SHORT message
 *
 * A short command message for those messages requiring only one parameter. This was established to reduce the bandwidth required of messages not requiring the as much parameterized data.
 */
struct COMMAND_SHORT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 31;
    static constexpr size_t LENGTH = 6;
    static constexpr size_t MIN_LENGTH = 6;
    static constexpr uint8_t CRC_EXTRA = 226;
    static constexpr auto NAME = "COMMAND_SHORT";


    uint8_t target_system; /*< System which should execute the command */
    uint8_t target_component; /*< Component which should execute the command, 0 for all components */
    uint16_t command; /*< Command ID, as defined by MAV_CMD enum. */
    uint8_t confirmation; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) */
    uint8_t param; /*< Parameter as defined by MAV_CMD enum. */


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
        ss << "  command: " << command << std::endl;
        ss << "  confirmation: " << +confirmation << std::endl;
        ss << "  param: " << +param << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << command;                       // offset: 0
        map << target_system;                 // offset: 2
        map << target_component;              // offset: 3
        map << confirmation;                  // offset: 4
        map << param;                         // offset: 5
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> command;                       // offset: 0
        map >> target_system;                 // offset: 2
        map >> target_component;              // offset: 3
        map >> confirmation;                  // offset: 4
        map >> param;                         // offset: 5
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
