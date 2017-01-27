#ifndef SENSOR_CAMERA_H
#define SENSOR_CAMERA_H

#include "data/i_topic_component_data_object.h"
#include "math.h"

namespace DataVehicleSensors
{

extern const char Camera_name[];
extern const MaceCore::TopicComponentStructure Camera_structure;

class SensorCamera : public Data::NamedTopicComponentDataObject<Camera_name, &Camera_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
    void updateCameraProperties();
public:

    double getFOV_Horizontal(){
        return(m_HFOVA);
    }
    double getFOV_Vertical(){
        return(m_HFOVA);
    }


    void setFocalLength(const double &focalLength){
        m_FocalLength = focalLength;
        this->updateCameraProperties();
    }
    double getFocalLength(){
        return(m_FocalLength);
    }


    void setImageWidth(const double &imageWidth){
        m_Image_Width = imageWidth;
    }
    double getImageWidth(){
        return(m_Image_Width);
    }


    void setImageHeight(const double &imageHeight){
        m_Image_Height = imageHeight;
    }
    double getImageHeight(){
        return(m_Image_Height);
    }


    void setSensorWidth(const double &sensorWidth){
        m_Sensor_Width = sensorWidth;
        this->updateCameraProperties();
    }
    double getSensorWidth(){
        return(m_Sensor_Width);
    }


    void setSensorHeight(const double &sensorHeight){
        m_Sensor_Height = sensorHeight;
        this->updateCameraProperties();
    }
    double getSensorHeight(){
        return(m_Sensor_Height);
    }


    void setImageRate(const double &rate){
        m_Image_Rate = rate;
    }
    double getImageRate(){
       return(m_Image_Rate);
    }

public:
//    bool operator == (const SensorCamera &rhs) {
//    }

//    bool operator != (const SensorCamera &rhs) {
//        return !(*this == rhs);
//    }

protected:
    double m_HFOVA; //value in radians
    double m_VFOVA; //value in radians
    double m_FocalLength; //value in mm
    double m_Image_Width; //value in pixels
    double m_Image_Height; //value in pixels
    double m_Sensor_Width; //value in mm
    double m_Sensor_Height; //value in mm
    double m_Image_Rate; //value in hz

    bool providedFOV;
};

}

#endif // SENSOR_CAMERA_H
