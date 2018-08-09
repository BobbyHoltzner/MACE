#ifndef MAP_CELL_H
#define MAP_CELL_H

#include <data/environment_time.h>
#include <maps/occupancy_definition.h>

#include <memory>

namespace mace {
namespace maps {

class MapCell
{
public:
    MapCell();

    MapCell(mace::maps::OccupiedResult cellValue);

    MapCell(mace::maps::OccupiedResult cellValue, double logOddsProbability);

    MapCell(mace::maps::OccupiedResult cellValue, double logOddsProbability, bool potentialTaskFlag);

    void setCellValue(const mace::maps::OccupiedResult &value) {
        this->occupiedResult = value;
    }

    void setCellValue(const mace::maps::OccupiedResult &value, const Data::EnvironmentTime &time) {
        this->occupiedResult = value;
        updateDiscoveryTime(time);
    }

    mace::maps::OccupiedResult getCellValue() { return occupiedResult; }

    void setLogOdds(const double &logOdds) {
        this->logOddsProbability = logOdds;
    }

    double getLogOdds() { return this->logOddsProbability; }

    void setPotentialTaskFlag(const bool &taskFlag) {
        this->potentialTaskFlag = taskFlag;
    }

    bool getPotentialTaskFlag() { return potentialTaskFlag; }


    void updateDiscoveryTime(const Data::EnvironmentTime &time) {
        this->discoveryTime = std::make_shared<Data::EnvironmentTime>(time);
    }

    std::shared_ptr<Data::EnvironmentTime> getDiscoveryTime() { return discoveryTime; }

    void updateLogOddsProbability(const OccupiedResult &occupiedResult, const double &p_d, const double &p_fa, const double &sigma);


private:
    mace::maps::OccupiedResult occupiedResult;
    double logOddsProbability;

    std::shared_ptr<Data::EnvironmentTime> discoveryTime;

    bool potentialTaskFlag;

};

} //end of namespace maps
} //end of namespace mace

#endif // MAP_CELL_H
