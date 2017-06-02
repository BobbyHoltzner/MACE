#ifndef ACTION_MISSION_COMMAND_H
#define ACTION_MISSION_COMMAND_H

#include <iostream>

#include "data/command_item_type.h"
#include "data/mission_command.h"
#include "data_generic_command_item/abstract_command_item.h"

namespace CommandItem {

class ActionMissionCommand : public AbstractCommandItem
{


public:
    virtual Data::CommandItemType getCommandType() const;

    virtual std::string getDescription() const;

    virtual bool hasSpatialInfluence() const;

public:
    ActionMissionCommand();
    ActionMissionCommand(const ActionMissionCommand &obj);
    ActionMissionCommand(const int &systemOrigin, const int &systemTarget);

public:

    void setMissionCommandType(const Data::MissionCommandAction &action)
    {
        this->missionCommand = action;
    }

    void setMissionStart()
    {
        this->missionCommand = Data::MissionCommandAction::MISSIONCA_START;
    }

    void setMissionPause()
    {
        this->missionCommand = Data::MissionCommandAction::MISSIONCA_PAUSE;
    }

    void setMissionResume()
    {
        this->missionCommand = Data::MissionCommandAction::MISSIONCA_RESUME;
    }

    Data::MissionCommandAction getMissionCommandAction() const
    {
        return missionCommand;
    }

public:
    void operator = (const ActionMissionCommand &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->missionCommand = rhs.missionCommand;
    }

    bool operator == (const ActionMissionCommand &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->missionCommand != rhs.missionCommand){
            return false;
        }
        return true;
    }

    bool operator != (const ActionMissionCommand &rhs) {
        return !(*this == rhs);
    }

private:
    Data::MissionCommandAction missionCommand;
};

} //end of namespace CommandItem

#endif // ACTION_MISSION_COMMAND_H
