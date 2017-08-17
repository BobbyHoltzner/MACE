#include "command_controller_externalLink.h"

namespace ExternalLink {


CommandController_ExternalLink::CommandController_ExternalLink(CommandController_Interface *cb):
    systemID(0), transmittingID(0),
    currentRetry(0), maxRetries(3), responseTimeout(5000),\
    currentCommsState(Data::ControllerCommsState::NEUTRAL),
    m_CB(NULL), prevTransmit(NULL), mLog(NULL)
{
    connectCallback(cb);
}

void CommandController_ExternalLink::updateLogging(const bool &toLog, const std::string &name)
{
    UNUSED(toLog);
    mLog = spdlog::get(name);
}

void CommandController_ExternalLink::updateIDS(const int &targetID, const int &originatingID)
{
    this->systemID = targetID;
    this->transmittingID = originatingID;
}

void CommandController_ExternalLink::clearPreviousTransmit()
{
    if(prevTransmit)
    {
        delete prevTransmit;
        prevTransmit = NULL;
    }
}

void CommandController_ExternalLink::receivedCommandACK(const mace_command_ack_t &cmdACK)
{
    m_LambdasToRun.push_back([this, cmdACK]{
        mTimer.stop();

        logCommandACK(cmdACK);

        if(mLog)
           mLog->error("Command Controller recevied an acknowledgement of type " + std::to_string(cmdACK.result) + " for command " + std::to_string(cmdACK.command) + ".");

        commandItemEnum type = prevTransmit->getType();
        switch(type)
        {
        case(commandItemEnum::COMMAND_SHORT):
        {
            PreviousCommand<mace_command_short_t> *tmp = static_cast<PreviousCommand<mace_command_short_t>*>(prevTransmit);
            mace_command_short_t prevCMD = tmp->getData();
            if(prevCMD.command == cmdACK.command)
            {
                clearPendingTasks();
                mToExit = true;
                currentCommsState = Data::ControllerCommsState::NEUTRAL;
            }
            break;
        }
        case(commandItemEnum::COMMAND_LONG):
        {
            PreviousCommand<mace_command_long_t> *tmp = static_cast<PreviousCommand<mace_command_long_t>*>(prevTransmit);
            mace_command_long_t prevCMD = tmp->getData();
            if(prevCMD.command == cmdACK.command)
            {
                clearPendingTasks();
                mToExit = true;
                currentCommsState = Data::ControllerCommsState::NEUTRAL;
            }
            break;
        }
        default:
        {

        }
        }
    });
}

mace_command_long_t CommandController_ExternalLink::initializeCommandLong()
{
    mace_command_long_t cmdLong;
    cmdLong.command = 0;
    cmdLong.confirmation = 0;
    cmdLong.param1 = 0.0;
    cmdLong.param2 = 0.0;
    cmdLong.param3 = 0.0;
    cmdLong.param4 = 0.0;
    cmdLong.param5 = 0.0;
    cmdLong.param6 = 0.0;
    cmdLong.param7 = 0.0;
    cmdLong.target_system = systemID;
    cmdLong.target_component = 0;
    return cmdLong;
}

mace_command_short_t CommandController_ExternalLink::initializeCommandShort()
{
    mace_command_short_t cmdShort;
    cmdShort.command = 0;
    cmdShort.confirmation = 0;
    cmdShort.param = 0.0;
    cmdShort.target_system = systemID;
    cmdShort.target_component = 0;
    return cmdShort;
}

void CommandController_ExternalLink::setSystemArm(const CommandItem::ActionArm &commandItem, const int &compID)
{
    if(mLog)
    {
        std::stringstream buffer;
        buffer << commandItem;

        mLog->debug("Command Controller is requesting the system to arm.");
        mLog->info(buffer.str());
    }

    mace_command_short_t cmd = initializeCommandShort();
    cmd.command = (uint8_t)CommandItem::COMMANDITEM::CI_ACT_ARM;
    cmd.target_system = commandItem.getTargetSystem();
    cmd.target_component = compID;
    cmd.param = commandItem.getRequestArm();

    clearPreviousTransmit();
    prevTransmit = new PreviousCommand<mace_command_short_t>(commandItemEnum::COMMAND_SHORT, cmd);

    currentCommsState = Data::ControllerCommsState::TRANSMITTING;
    currentRetry = 0;
    this->start();
    mTimer.start();

    m_CB->cbiCommandController_transmitCommand(cmd);
}

void CommandController_ExternalLink::setSystemTakeoff(const CommandItem::SpatialTakeoff &commandItem, const int &compID)
{
    if(mLog)
    {
        std::stringstream buffer;
        buffer << commandItem;

        mLog->debug("Command Controller is requesting the system to takeoff.");
        mLog->info(buffer.str());
    }

    mace_command_long_t cmd = initializeCommandLong();
    cmd.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_TAKEOFF;
    cmd.target_system = commandItem.getTargetSystem();
    cmd.target_component = compID;
    Data::CoordinateFrameType cf = commandItem.position->getCoordinateFrame();

    if(commandItem.position->has2DPositionSet())
    {
        cmd.param5 = commandItem.position->getX();
        cmd.param6 = commandItem.position->getY();
    }
    cmd.param7 = commandItem.position->getZ();

    clearPreviousTransmit();
    prevTransmit = new PreviousCommand<mace_command_long_t>(commandItemEnum::COMMAND_LONG, cmd);

    currentCommsState = Data::ControllerCommsState::TRANSMITTING;
    currentRetry = 0;
    this->start();
    mTimer.start();

    m_CB->cbiCommandController_transmitCommand(cmd);
}

void CommandController_ExternalLink::setSystemLand(const CommandItem::SpatialLand &commandItem, const int &compID)
{
    if(mLog)
    {
        std::stringstream buffer;
        buffer << commandItem;

        mLog->debug("Command Controller is requesting the system to land.");
        mLog->info(buffer.str());
    }

    mace_command_long_t cmd = initializeCommandLong();
    cmd.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND;
    cmd.target_system = commandItem.getTargetSystem();
    cmd.target_component = compID;

    if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
    {
        cmd.param5 = commandItem.position->getX() * pow(10,7);
        cmd.param6 = commandItem.position->getY() * pow(10,7);
        cmd.param7 = commandItem.position->getZ() * 1000;
    }

    clearPreviousTransmit();
    prevTransmit = new PreviousCommand<mace_command_long_t>(commandItemEnum::COMMAND_LONG, cmd);

    currentCommsState = Data::ControllerCommsState::TRANSMITTING;
    currentRetry = 0;
    mToExit = false;
    this->start();
    mTimer.start();

    m_CB->cbiCommandController_transmitCommand(cmd);
}

void CommandController_ExternalLink::setSystemRTL(const CommandItem::SpatialRTL &commandItem, const int &compID)
{
    if(mLog)
        mLog->debug("Command Controller is requesting the system to RTL.");

    mace_command_short_t cmd = initializeCommandShort();
    cmd.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH;
    cmd.target_system = commandItem.getTargetSystem();
    cmd.target_component = compID;

    clearPreviousTransmit();
    prevTransmit = new PreviousCommand<mace_command_short_t>(commandItemEnum::COMMAND_SHORT, cmd);

    currentCommsState = Data::ControllerCommsState::TRANSMITTING;
    currentRetry = 0;
    this->start();
    mTimer.start();

    m_CB->cbiCommandController_transmitCommand(cmd);
}

void CommandController_ExternalLink::setSystemMissionCommand(const CommandItem::ActionMissionCommand &commandItem, const int &compID)
{
    if(mLog)
        mLog->debug("Command Controller is requesting to set the system mission command.");

    mace_command_short_t cmd = initializeCommandShort();
    cmd.command = (uint16_t)CommandItem::COMMANDITEM::CI_ACT_MISSIONCOMMAND;
    cmd.target_system = commandItem.getTargetSystem();
    cmd.target_component = compID;
    cmd.param = (uint8_t)commandItem.getMissionCommandAction();

    clearPreviousTransmit();
    prevTransmit = new PreviousCommand<mace_command_short_t>(commandItemEnum::COMMAND_SHORT, cmd);

    currentCommsState = Data::ControllerCommsState::TRANSMITTING;
    currentRetry = 0;
    this->start();
    mTimer.start();

    m_CB->cbiCommandController_transmitCommand(cmd);
}

void CommandController_ExternalLink::run()
{
    while(true)
    {
        if(mToExit == true) {
            clearPendingTasks();
            clearPreviousTransmit();
            mTimer.stop();
            break;
        }

        this->RunPendingTasks();

        //The current state we can find out how much time has passed.
        //If one of the lambda expressions has fired the clock shoud
        //be reset right at the end, thus making this value small and
        //improbable the next function will fire
        double timeElapsed = mTimer.elapsedMilliseconds();

        if(timeElapsed > responseTimeout)
        {
            commandItemEnum type = prevTransmit->getType();
            currentRetry++;

            switch(currentCommsState)
            {
            case Data::ControllerCommsState::NEUTRAL:
            {
                //This case we should terminate this because there is nothing we should be doing apparently
                clearPreviousTransmit();
                mTimer.stop();
                mToExit = true;
             break;
            }
            case Data::ControllerCommsState::TRANSMITTING:
            {

                if(type == commandItemEnum::COMMAND_SHORT)
                {
                    PreviousCommand<mace_command_short_t> *tmp = static_cast<PreviousCommand<mace_command_short_t>*>(prevTransmit);
                    mace_command_short_t msgTransmit = tmp->getData();
                    if(mLog)
                        mLog->error("Command Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommandItemEnumString(type) + " of type " + std::to_string(msgTransmit.command) + ".");
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiCommandController_transmitCommand(msgTransmit);
                }
                else if(type == commandItemEnum::COMMAND_LONG)
                {
                    PreviousCommand<mace_command_long_t> *tmp = static_cast<PreviousCommand<mace_command_long_t>*>(prevTransmit);
                    mace_command_long_t msgTransmit = tmp->getData();
                    if(mLog)
                        mLog->error("Command Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommandItemEnumString(type) + " of type " + std::to_string(msgTransmit.command) + ".");
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiCommandController_transmitCommand(msgTransmit);
                }
                break;
            }
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void CommandController_ExternalLink::logCommandACK(const mace_command_ack_t &cmdACK)
{
    std::string result = "";
    std::string base = "Command " + std::to_string(cmdACK.command) + " was ";

    switch(cmdACK.result)
    {
    case MAV_RESULT_ACCEPTED:
        result = "accepted.";
        break;
    case MAV_RESULT_DENIED:
        result = "denied.";
        break;
    case MAV_RESULT_FAILED:
        result = "failed.";
        break;
    case MAV_RESULT_TEMPORARILY_REJECTED:
        result = "temporarily rejected.";
        break;
    case MAV_RESULT_UNSUPPORTED:
        result = "unsupported.";
        break;
    default:
        result = "unknown.";
    }

    base = base + result;
    //mLog->info(base);
}


} //end of namespace DataInterface_ExternalLink
