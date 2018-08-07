#ifndef SENSOR_CIRCULAR_CAMERA_H
#define SENSOR_CIRCULAR_CAMERA_H

#include <string>
#include "data/i_topic_component_data_object.h"
#include "math.h"

namespace DataVehicleSensors
{

extern const char Circular_Camera_name[];
extern const MaceCore::TopicComponentStructure Circular_Camera_structure;

class SensorCircularCamera : public Data::NamedTopicComponentDataObject<Circular_Camera_name, &Circular_Camera_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:

    void setCameraName(const std::string &cameraName){
        this->cameraName = cameraName;
    }
    std::string getCameraName() const{
        return(cameraName);
    }

    void setViewHalfAngle(const double &viewHalfAngle) {
        this->viewHalfAngle = viewHalfAngle;
    }
    double getViewHalfAngle() const{
        return(viewHalfAngle);
    }


public:
    bool operator == (const SensorCircularCamera &rhs) {
        if(this->viewHalfAngle != rhs.viewHalfAngle){
            return false;
        }

        return true;
    }

    bool operator != (const SensorCircularCamera &rhs) {
        return !(*this == rhs);
    }

protected:
    std::string cameraName;
    double viewHalfAngle; //value in degrees
};

}

#endif // SENSOR_CIRCULAR_CAMERA_H
