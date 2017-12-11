#ifndef COMMAND_CONTROLLER_EXTERNALLINK_H
#define COMMAND_CONTROLLER_EXTERNALLINK_H

#include <iostream>
#include <QDate>
#include "spdlog/spdlog.h"

#include "common/common.h"
#include "mace.h"

#include "data/controller_comms_state.h"
#include "data/threadmanager.h"
#include "data/timer.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_interface_MACE/generic/helper_previous_command_mace.h"

using namespace DataInterface_MACE;

namespace ExternalLink {

class CommandController_Interface
{
public:
    virtual void cbiCommandController_transmitCommand(const mace_command_short_t &cmd) = 0;
    virtual void cbiCommandController_transmitCommand(const mace_command_long_t &cmd) = 0;
    virtual void cbiCommandController_transmitCommand(const mace_command_system_mode_t &cmd) = 0;
    virtual void cbiCommandController_CommandACK(const mace_command_ack_t &ack) = 0;
};

class CommandController_ExternalLink : public Thread
{
public:
    CommandController_ExternalLink(CommandController_Interface *cb);

    ~CommandController_ExternalLink() {
        std::cout << "Destructor on the mavlink command controller" << std::endl;
        mToExit = true;
    }

    void updateLogging(const bool &toLog, const std::string &name);

    void connectCallback(CommandController_Interface *cb)
    {
        m_CB = cb;
    }

    void updateIDS(const int &targetID, const int &originatingID);

    void run();

    void receivedCommandACK(const mace_command_ack_t &cmdACK);
    void receivedModeACK(const mace_system_mode_ack_t &modeACK);

    void setSystemArm(const CommandItem::ActionArm &commandItem, const int &compID = 0);
    void setSystemTakeoff(const CommandItem::SpatialTakeoff &commandItem, const int &compID = 0);
    void setSystemLand(const CommandItem::SpatialLand &commandItem, const int &compID = 0);
    void setSystemRTL(const CommandItem::SpatialRTL &commandItem, const int &compID = 0);
    void setSystemMode(const CommandItem::ActionChangeMode &commandItem, const int &compID = 0);

    void setSystemMissionCommand(const CommandItem::ActionMissionCommand &commandItem, const int &compID = 0);

    Data::ControllerCommsState getCommsState() const
    {
        return this->currentCommsState;
    }

private:
    void logCommandACK(const mace_command_ack_t &cmdACK);
    void clearPreviousTransmit();

    mace_command_long_t initializeCommandLong();
    mace_command_short_t initializeCommandShort();
private:
    int systemID;
    int transmittingID;

    Timer mTimer;
    int currentRetry;
    int maxRetries;
    int responseTimeout;

private:
    std::shared_ptr<spdlog::logger> mLog;

    CommandController_Interface *m_CB;
    PreviousTransmissionBase<commandItemEnum> *prevTransmit;

    Data::ControllerCommsState currentCommsState;

protected:
    std::list<std::function<void()>> m_LambdasToRun;

    void clearPendingTasks()
    {
        m_LambdasToRun.clear();
    }

    void RunPendingTasks() {
        while(m_LambdasToRun.size() > 0) {
            auto lambda = m_LambdasToRun.front();
            m_LambdasToRun.pop_front();
            lambda();
        }
    }

};


} //end of namespace ExternalLink
#endif // COMMAND_CONTROLLER_MAVLINK_H
