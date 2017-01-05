#ifndef POSITIONALAID_H
#define POSITIONALAID_H

#include "global_position.h"
#include "local_position.h"

namespace Data{

/**
 * @brief The PositionalAid class provides functions to convert between different representaions of position.
 *
 * @see LocalPosition, GlobalPosition
 */

class PositionalAid
{
public:

    /**
     * @brief Construct a new DynamicsAid.
     */
    PositionalAid();

    /**
     * @brief Destroy the PositionalAid.
     */
    ~PositionalAid();

    static void TransformToLocalPosition(const GlobalPosition &globalOrigin, const GlobalPosition &globalPosition, LocalPosition &localPosition);

    static void TransformToGlobalPosition(const GlobalPosition &globalOrigin, const LocalPosition &localPosition, GlobalPosition &globalPosition);

};

} //end of namespace Data

#endif // POSITIONALAID_H
