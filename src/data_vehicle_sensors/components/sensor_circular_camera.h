#ifndef SENSOR_CIRCULAR_CAMERA_H
#define SENSOR_CIRCULAR_CAMERA_H

#include <string>
#include "data/i_topic_component_data_object.h"

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
    std::string getCameraName() const {
        return(cameraName);
    }

    void setViewHalfAngle(const double &halfAngle) {
        this->viewHalfAngle = halfAngle;
    }
    double getViewHalfAngle() const {
        return(viewHalfAngle);
    }

    void setAlphaAttenuation(const double &alpha) {
        this->alphaAttenuation = alpha;
    }
    double getAlphaAttenuation() const {
        return(alphaAttenuation);
    }

    void setBetaAttenuation(const double &beta) {
        this->betaAttenuation = beta;
    }
    double getBetaAttenuation() const {
        return(betaAttenuation);
    }

    void setCertainRange(const double &range) {
        this->certainRangePercent = range;
    }
    double getCertainRange() const {
        return(certainRangePercent);
    }

    void setProbDetection(const double &probDetection) {
        this->p_d = probDetection;
    }
    double getProbDetection() const {
        return(p_d);
    }

    void setProbFalseAlarm(const double &probFalseAlarm) {
        this->p_fa = probFalseAlarm;
    }
    double getProbFalseAlarm() const {
        return(p_fa);
    }


    double attenuatedDiskConfidence(const double &distanceToSensorOrigin, const double &radius);


public:
    bool operator == (const SensorCircularCamera &rhs) {
        if(this->cameraName != rhs.cameraName){
            return false;
        }
        if(this->viewHalfAngle != rhs.viewHalfAngle){
            return false;
        }
        if(this->alphaAttenuation != rhs.alphaAttenuation){
            return false;
        }
        if(this->betaAttenuation != rhs.betaAttenuation){
            return false;
        }
        if(this->certainRangePercent != rhs.certainRangePercent){
            return false;
        }
        if(this->p_d != rhs.p_d){
            return false;
        }
        if(this->p_fa != rhs.p_fa){
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

    double alphaAttenuation;
    double betaAttenuation;
    // Between 0-1; If 0, our attenuation will start immediately. If > 0, attenuation starts certainRange % from the center
    double certainRangePercent;
    // Probability of Detection (0->1)
    double p_d;
    // Probability of False Alarm (0->1)
    double p_fa;
};

}

#endif // SENSOR_CIRCULAR_CAMERA_H
