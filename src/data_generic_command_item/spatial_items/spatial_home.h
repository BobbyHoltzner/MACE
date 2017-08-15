#ifndef SPATIAL_HOME_H
#define SPATIAL_HOME_H

#include <iostream>
#include <sstream>

#include "mace.h"

#include "data/command_item_type.h"
#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_state_item/base_3d_position.h"
#include "spatial_abstract_position.h"

namespace CommandItem {

//!
//! \brief The SpatialHome class
//!
class SpatialHome : public AbstractCommandItem
{
public:
    //!
    //! \brief SpatialHome The default constructor for a SpatialHome commandItem. This will default the
    //! originating and target systems to 0. Additionally, the position will default to 0,0,0 with
    //! a global relative alt coordinate frame definition.
    //!
    SpatialHome();

    ~SpatialHome();

    //!
    //! \brief SpatialHome A default copy constructor of a SpatialHome commandItem object.
    //! \param obj of type SpatialHome that the data shall be copied from.
    //!
    SpatialHome(const SpatialHome &obj);

    //!
    //! \brief SpatialHome An overloaded default constructor for a SpatialHome commandItem.
    //! \param systemOrigin The ID value of the system that initiated the command item.
    //! \param systemTarget The ID value of the system that is the intended recipient of the command item.
    //! A developer should be aware that this value defaults to 0 if no arguments are provided.
    //!
    SpatialHome(const int &systemOrigin, const int &systemTarget = 0);

    mace_home_position_t getMACECommsObject() const;
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    //!
    //! \brief getCommandType returns the type of the object that this command type is.
    //! \return Data::CommandType resolving the type of command this object is.
    //!
    virtual Data::CommandItemType getCommandType()const;

    //!
    //! \brief getDescription
    //! \return string describing the command item. This may be useful for setting up options in a
    //! GUI or somewhere a display needs to interface and decisions have to be made describing what
    //! would happen when issuing such a command.
    //!
    virtual std::string getDescription()const;

    //!
    //! \brief hasSpatialInfluence returns a boolean reflecting whether or not the commandItem has
    //! a direct influence over a vehicles position. This is useful for determining flight times,
    //! position elements, or rendering objects on a GUI.
    //! \return false if the command does not have an affect over the vehicles position directly.
    //! For example, change speed has no influence over a vehicles position.
    //!
    //!
    virtual bool hasSpatialInfluence()const;


public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    SpatialHome& operator = (const SpatialHome &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->position = rhs.position;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return 
    //!
    bool operator == (const SpatialHome &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->position != rhs.position)
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return 
    //!
    bool operator != (const SpatialHome &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const SpatialHome& t);

public:
    //!
    //! \brief position
    //!
    DataState::Base3DPosition *position;
};

} //end of namespace MissionItem

#endif // SPATIAL_HOME_H
