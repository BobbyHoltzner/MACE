#include "global_position.h"

#include <math.h>

namespace DataVehicleGeneric
{


const char GlobalPosition_name[] = "global_position";
const MaceCore::TopicComponentStructure GlobalPosition_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("latitude");
    structure.AddTerminal<double>("longitude");
    structure.AddTerminal<std::unordered_map<int, double>>("altitude");

    return structure;
}();




MaceCore::TopicDatagram GlobalPosition::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("latitude", latitude);
    datagram.AddTerminal<double>("longitude", longitude);
    datagram.AddTerminal<std::unordered_map<int, double>>("altitude", altitude);

    return datagram;
}


void GlobalPosition::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    latitude = datagram.GetTerminal<double>("latitude");
    longitude = datagram.GetTerminal<double>("longitude");
    altitude = datagram.GetTerminal<std::unordered_map<int, double>>("altitude");
}


/**
 * @brief GlobalPosition::NewPositionFromHeadingBearing
 * @param distance
 * @param bearing
 * @param degreesFlag
 * @param newPosition
 */
GlobalPosition GlobalPosition::NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag)
{
    double earthRadius = 6371000; //approximate value in meters
    double latitudeRad = convertDegreesToRadians(latitude);
    double longitudeRad = convertDegreesToRadians(longitude);
    double bearingRad = 0.0;

    if(degreesFlag == true){
        bearingRad = convertDegreesToRadians(bearing);
    }else{
        bearingRad = bearing;
    }
    double distanceRatio = distance / earthRadius;

    double newLat = asin(sin(latitudeRad) * cos(distanceRatio) + cos(latitudeRad) * sin(distanceRatio) * cos(bearingRad));
    double newLon = longitudeRad + atan2(sin(bearingRad) * sin(distanceRatio) * cos(latitudeRad),
                                         cos(distanceRatio) - sin(latitudeRad) * sin(newLat));

    GlobalPosition newPos;
    newPos.latitude = convertRadiansToDegrees(newLat);
    newPos.longitude = convertRadiansToDegrees(newLon);
    newPos.altitude = altitude;

    return newPos;
}


/**
 * @brief GlobalPosition::bearingBetween
 * @param position
 * @return
 */
double GlobalPosition::bearingBetween(const GlobalPosition &position)
{
    double deltaLatitude = convertDegreesToRadians(position.latitude - this->latitude);
    double deltaLongitude = convertDegreesToRadians(position.longitude - this->longitude);

    double tmpY = sin(deltaLongitude) * cos(convertDegreesToRadians(position.latitude));
    double tmpX = cos(convertDegreesToRadians(this->latitude)) * sin(convertDegreesToRadians(position.latitude)) -
            sin(convertDegreesToRadians(this->latitude)) * sin(convertDegreesToRadians(position.latitude)) *
            cos(deltaLongitude);
    double bearing = atan2(tmpY,tmpX);

    return convertRadiansToDegrees(bearing);
}

/**
 * @brief GlobalPosition::initialBearing
 * @param position
 * @return
 */
double GlobalPosition::initialBearing(const GlobalPosition &position){
    throw std::runtime_error("Not Implimented");
    return 0.0;
    //return (bearingBetween(position) + 360.0) % 360.0;
}

/**
 * @brief GlobalPosition::finalBearing
 * @param position
 * @return
 */
double GlobalPosition::finalBearing(const GlobalPosition &position){
    throw std::runtime_error("Not Implimented");
    return 0.0;
    //return (bearingBetween(position) + 180.0) % 360.0;
}

/**
 * @brief GlobalPosition::distanceBetween2D
 * @param position
 * @return
 */
double GlobalPosition::distanceBetween2D(const GlobalPosition &position)
{
    double earthRadius = 6371000; //approximate value in meters
    double deltaLatitude = convertDegreesToRadians(position.latitude - this->latitude);
    double deltaLongitude = convertDegreesToRadians(position.longitude - this->longitude);

    double tmpA = sin(deltaLatitude/2) * sin(deltaLatitude/2) +
            cos(this->latitude) * cos(position.latitude) *
            sin(deltaLongitude/2) * sin(deltaLongitude/2);

    double tmpC = 2 * atan2(sqrt(tmpA),sqrt(1-tmpA));

    double distance = earthRadius * tmpC;

    return distance;
}

/**
 * @brief GlobalPosition::distanceBetween3D
 * @param position
 * @return
 */
double GlobalPosition::distanceBetween3D(const GlobalPosition &position, const int &altitudeCode)
{
    double distance2D = this->distanceBetween2D(position);
    double deltaAltitude = fabs(this->altitude.at(altitudeCode) - position.altitude.at(altitudeCode));
    return(sqrt(deltaAltitude * deltaAltitude + distance2D * distance2D));
}


/**
 * @brief GlobalPosition::convertDegreesToRadians
 * @param degrees
 * @return
 */
double GlobalPosition::convertDegreesToRadians(const double &degrees)
{
    double pi = 3.14159265358979323846;
    double radians = degrees * (pi/180.0);
    return radians;
}

/**
 * @brief GlobalPosition::convertRadiansToDegrees
 * @param radians
 * @return
 */
double GlobalPosition::convertRadiansToDegrees(const double &radians)
{
    double pi = 3.14159265358979323846;
    double degrees = radians * (180.0/pi);
    return degrees;
}

}
