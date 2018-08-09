#include "map_cell.h"

#include <math.h>

namespace mace{
namespace maps{

MapCell::MapCell() :
    occupiedResult(mace::maps::OccupiedResult::NO_DATA), logOddsProbability(0.0), potentialTaskFlag(false)
{
    discoveryTime = std::make_shared<Data::EnvironmentTime>();
}

MapCell::MapCell(mace::maps::OccupiedResult occupiedResult) :
    occupiedResult(occupiedResult), logOddsProbability(0.0), potentialTaskFlag(false)
{
    discoveryTime = std::make_shared<Data::EnvironmentTime>();
}

MapCell::MapCell(mace::maps::OccupiedResult occupiedResult, double confidence) :
    occupiedResult(occupiedResult), logOddsProbability(confidence), potentialTaskFlag(false)
{
    discoveryTime = std::make_shared<Data::EnvironmentTime>();
}

MapCell::MapCell(mace::maps::OccupiedResult occupiedResult, double confidence, bool potentialTaskFlag) :
    occupiedResult(occupiedResult), logOddsProbability(confidence), potentialTaskFlag(potentialTaskFlag)
{
    discoveryTime = std::make_shared<Data::EnvironmentTime>();
}


void MapCell::updateLogOddsProbability(const OccupiedResult &occupiedResult, const double &p_d, const double &p_fa, const double &sigma) {
    double logOdds = 0.0;

    if(occupiedResult == OccupiedResult::OCCUPIED) {
        logOdds = p_d; // logOdds_occupied
    }
    else if(occupiedResult == OccupiedResult::NOT_OCCUPIED) {
        logOdds = -p_fa; // logOdds_free
    }
    else if(occupiedResult == OccupiedResult::NO_DATA) {
        logOdds = 0.0;
    }
    else if(occupiedResult == OccupiedResult::OUTSIDE_ENVIRONMENT) {
        logOdds = 0.0;
    }

    // Calculate log odds:
    logOdds = this->getLogOdds() + logOdds*sigma;

    // Update our confidence with the new value:
    logOddsProbability = logOdds;

    // TODO: Either here or in the sensors module:
    //          - Update potential Task flag based on result and log odds value
}

} //end of namespace maps
} //end of namespace mace
