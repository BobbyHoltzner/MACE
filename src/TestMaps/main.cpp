#include <QCoreApplication>

#include "base/pose/point_2d.h"
#include "base/pose/point_3d.h"

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
    mace::pose::Point2D point2D(0.1,0.1);
    mace::pose::Point3D point3D(0,0,1.0);
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
