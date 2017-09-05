#include <QCoreApplication>

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"
#include "base/pose/cartesian_position_temp.h"

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


    mace::misc::Data2D data2D(0.5,0.5);
    mace::misc::Data3D data3D(1.0,1.0,1.0);

    mace::misc::Data3D newData3D(data2D);

    CartesianPosition<mace::misc::Data2D> cartesian2D_1;
    CartesianPosition<mace::misc::Data2D> cartesian2D_2;
    CartesianPosition<mace::misc::Data3D> cartesian3D_1;

    std::cout<<cartesian2D_1.distanceTo(cartesian2D_2)<<std::endl;
    std::cout<<cartesian3D_1.distanceTo(cartesian2D_1)<<std::endl;

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
