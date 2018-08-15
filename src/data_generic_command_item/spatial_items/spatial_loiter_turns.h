#ifndef SPATIAL_LOITER_TURNS_H
#define SPATIAL_LOITER_TURNS_H

#include <iostream>
#include <iomanip>
#include <sstream>

#include "mace.h"

#include "abstract_spatial_position.h"

#include "data_generic_command_item/command_item_type.h"
#include "data/loiter_direction.h"
#include "data_generic_state_item/base_3d_position.h"

#include "data_generic_command_item/abstract_command_item.h"

namespace CommandItem {

class SpatialLoiter_Turns : public AbstractCommandItem, public AbstractSpatialPosition
{

public:
    SpatialLoiter_Turns();
    SpatialLoiter_Turns(const SpatialLoiter_Turns &obj);
    SpatialLoiter_Turns(const int &originatingSystem, const int &systemTarget = 0);

public:

    //!
    //! \brief getCommandType returns the type of the object that this command type is.
    //! \return Data::CommandType resolving the type of command this object is.
    //!
    COMMANDITEM getCommandType() const override;

    //!
    //! \brief getDescription
    //! \return string describing the command item. This may be useful for setting up options in a
    //! GUI or somewhere a display needs to interface and decisions have to be made describing what
    //! would happen when issuing such a command.
    //!
    std::string getDescription() const override;

    //!
    //! \brief hasSpatialInfluence returns a boolean reflecting whether or not the commandItem has
    //! a direct influence over a vehicles position. This is useful for determining flight times,
    //! position elements, or rendering objects on a GUI.
    //! \return false if the command does not have an affect over the vehicles position directly.
    //! For example, change speed has no influence over a vehicles position.
    //!
    bool hasSpatialInfluence() const override;

    //!
    //! \brief getClone
    //! \return
    //!
    std::shared_ptr<AbstractCommandItem> getClone() const override;

    /**
     * @brief getClone
     * @param state
     */
    void getClone(std::shared_ptr<AbstractCommandItem> &command) const override;

public:
    void operator = (const SpatialLoiter_Turns &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        AbstractSpatialPosition::operator =(rhs);
        this->direction = rhs.direction;
        this->radius = rhs.radius;
        this->turns = rhs.turns;
    }

    bool operator == (const SpatialLoiter_Turns &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(!AbstractSpatialPosition::operator ==(rhs))
        {
            return false;
        }
        if(this->direction != rhs.direction)
        {
            return false;
        }
        if(this->radius != rhs.radius)
        {
            return false;
        }
        if(this->turns != rhs.turns)
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialLoiter_Turns &rhs) {
        return !(*this == rhs);
    }

public:
    Data::LoiterDirection direction;
    double radius;
    double turns;
};

} //end of namespace MissionItem

#endif // SPATIAL_LOITER_TURNS_H
