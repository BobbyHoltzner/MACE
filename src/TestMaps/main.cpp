#include <QCoreApplication>

#include "base/pose/point_2d.h"
#include "base/pose/point_3d.h"
#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"
#include "base/pose/cartesian_position_template.h"

#include <iostream>

class StateData{
public:

    StateData():
        value(4.5)
    {

    }
    StateData(const StateData &copy)
    {
        this->value = copy.value;
    }

    StateData& operator =(const StateData &rhs)
    {
        this->value = rhs.value;
        return *this;
    }

    double getValue() const {
      this->value;
    }

    void setValue(const double &newValue){
        this->value = newValue;
    }

private:
    double value;
};

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    using namespace mace::pose;
    Point2D point2D(0.1,0.1);
    Point3D point3D(0,0,1.0);

    CartesianPosition_Temp<Point2D> newCartesian2D;
    CartesianPosition_Temp<Point2D> newCartesian2D_;
    CartesianPosition_Temp<Point3D> newCartesian3D;

    newCartesian2D.distanceTo(newCartesian2D_);
    newCartesian2D.distanceTo(newCartesian3D);
    newCartesian3D.distanceTo(newCartesian3D);
    newCartesian3D.distanceTo(newCartesian2D);
    CartesianPosition_2D cPoint2d;
    cPoint2d.position.set2DPosition(0,0);

    CartesianPosition_2D cPoint2d_2;
    cPoint2d_2.position.set2DPosition(1,1);

    if(cPoint2d_2 < cPoint2d)
        std::cout<<"It is greater"<<std::endl;
    else
        std::cout<<"It is not greater"<<std::endl;

    cPoint2d = cPoint2d_2;

    CartesianPosition_3D cPoint3d(cPoint2d);



//    testPoint2d = cPoint2d;


    //cPoint2d.distanceTo(cPoint3d);

    cPoint2d.distanceTo(cPoint2d);

    cPoint3d.distanceTo(cPoint3d);

//    CartesianPosition_Temp<Point3D> newPoint(cPoint2d);
//    CartesianPosition_2D cPoint2d;
//    cPoint2d.set2DPosition(10.0,20.0);
//    CartesianPosition_Temp cPoint3d(cPoint2d);

    //cPoint2d.getX();

    if(point2D < point3D)
    {
        std::cout<<"The values are equal"<<std::endl;
    }else
    {
        std::cout<<"The values are not equal"<<std::endl;
    }

    if(point3D < point2D)
    {
        std::cout<<"The values are equal"<<std::endl;
    }else
    {
        std::cout<<"The values are not equal"<<std::endl;
    }

    //mace::pose::CartesianPosition<mace::pose::Point3D> pos;
//    Maps::Dynamic2DGrid<StateData> newGrid;
//    int xInd = newGrid.indexFromXPos(5.3);
//    int yInd = newGrid.indexFromYPos(3.9);
//    double value = -10.0;
//    int counter = 0;
//    while(value <= 10.0)
//    {
//        std::cout<<"The value here is: "<<value<<"at position: "<<counter<<"."<<std::endl;
//        counter++;
//        value += 0.5;
//    }
    return a.exec();
}
