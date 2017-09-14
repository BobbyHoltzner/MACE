#include <QCoreApplication>

#include "maps/bounded_2d_grid.h"
#include "base/geometry/cell_2DC.h"

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


    mace::pose::Position<mace::pose::CartesianPosition_2D> pos1("Node1",-10,-10);
    mace::pose::Position<mace::pose::CartesianPosition_2D> pos2("Node2",-10,10);
    mace::pose::Position<mace::pose::CartesianPosition_2D> pos3("Node3",0,10);
    mace::pose::Position<mace::pose::CartesianPosition_2D> pos4("Node4",0,-10);

    mace::pose::Position<mace::pose::CartesianPosition_2D> pos5("Node5",10,10);
    mace::pose::Position<mace::pose::CartesianPosition_2D> pos6("Node6",10,-10);

    mace::geometry::Cell_2DC polygon3;
    polygon3.appendVertex(pos1);
    polygon3.appendVertex(pos2);
    polygon3.appendVertex(pos5);
    polygon3.appendVertex(pos6);


    mace::maps::Bounded2DGrid newGrid(polygon3,2,2);

    mace::geometry::Cell_2DC polygon1;
    polygon1.appendVertex(pos1);
    polygon1.appendVertex(pos2);
    polygon1.appendVertex(pos3);
    polygon1.appendVertex(pos4);

    mace::pose::Position<mace::pose::CartesianPosition_2D> pos10("Test",4,-10);
    std::cout<<"Does it contain this point: "<<polygon1.contains(pos10)<<std::endl;

//    std::list<mace::pose::Position<mace::pose::CartesianPosition_2D>*> nodeList = m_dataGrid->getBoundedDataList();
//    polygon1.insertNodes(nodeList,true);

//    mace::pose::Position<mace::pose::CartesianPosition_2D> point1("TEST1",-1.0,-1.0);
//    mace::pose::Position<mace::pose::CartesianPosition_2D> point2("TEST",-1.0,1.0);
//    mace::pose::Position<mace::pose::CartesianPosition_2D> point3("TEST1",1.0,1.0);
//    mace::pose::Position<mace::pose::CartesianPosition_2D> point4("TEST",1.0,-1.0);

//    mace::geometry::Polygon_2DC polygon;
//    polygon.appendVertex(point1);
//    polygon.appendVertex(point2);
//    polygon.appendVertex(point3);
//    polygon.appendVertex(point4);

//    mace::maps::Bounded2DGrid newGrid;
//    double x = 0.0;
//    double y = 0.0;
//    std::vector<mace::maps::Test*> inB = newGrid.setBoundingPolygon(polygon);
//    mace::maps::Test* individual = inB[0];
//    individual->pos.updatePosition(14.2,19.4);
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
