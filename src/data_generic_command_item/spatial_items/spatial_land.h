#ifndef SPATIAL_LAND_H
#define SPATIAL_LAND_H

#include <iostream>
#include <iomanip>
#include <sstream>

#include "mace.h"

#include "abstract_spatial_position.h"

#include "data_generic_command_item/command_item_type.h"

#include "data_generic_state_item/base_3d_position.h"

#include "data_generic_command_item/abstract_command_item.h"

namespace CommandItem {

class SpatialLand : public AbstractCommandItem, public AbstractSpatialPosition
{

public:
    SpatialLand();
    SpatialLand(const SpatialLand &obj);
    SpatialLand(const int &systemOrigin, const int &systemTarget = 0);

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
    AbstractCommandItem* getClone() const override;

    //!
    //! \brief getClone
    //! \param command
    //!
    void getClone(AbstractCommandItem** command) const override;

public:
    bool getLandFlag() const;
    void setLandFlag(const bool &landFlag);

public:
    void operator = (const SpatialLand &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        AbstractSpatialPosition::operator =(rhs);
    }

    bool operator == (const SpatialLand &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(!AbstractSpatialPosition::operator ==(rhs))
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialLand &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const SpatialLand& t);
};

} //end of namespace MissionItem

#endif // SPATIAL_LAND_H
