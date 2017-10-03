#include <QCoreApplication>

#include "base/state_space/cartesian_2D_space.h"

#include "planners/rrt_base.h"
#include "planners/nearest_neighbor_flann.h"

#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>

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
    using namespace mace::state_space;

//    using namespace mace::nn;

//    mace::planners_sampling::RRTBase newBase();
//    newBase.setNearestNeighbor<NearestNeighbor_FLANNLinear>();
//    mace::state_space::State* sampleState = space->getNewState();

//    mace::state_space::Cartesian2DSpace_Sampler sampler(space);
//    sampler.sampleUniform(sampleState);

    mace::state_space::Cartesian2DSpacePtr space = std::make_shared<mace::state_space::Cartesian2DSpace>();
    space->bounds.setBounds(0,10,0,10);
    mace::state_space::Cartesian2DSpace_SamplerPtr sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(space.get());

    mace::state_space::SpaceInformationPtr spaceInfo = std::make_shared<mace::state_space::SpaceInformation>(space);
    spaceInfo->setStateSampler(sampler);

    mace::planners_sampling::RRTBase rrt(spaceInfo);
    mace::state_space::GoalState* begin = new mace::state_space::GoalState(space);
    begin->setState(new mace::pose::CartesianPosition_2D(0,0));
    mace::state_space::GoalState* end = new mace::state_space::GoalState(space,1.0);
    end->setState(new mace::pose::CartesianPosition_2D(10,10));
    end->setRadialRegion(1.0);

    rrt.setPlanningParameters(begin,end);

    rrt.setNearestNeighbor<mace::nn::NearestNeighbor_FLANNLinear<mace::planners_sampling::RootNode*>>();
    std::vector<mace::state_space::State*> solution = rrt.solve();
    std::cout<<"The solution looks like this: "<<std::endl;
    for (int i = 0; i < solution.size(); i++)
    {
        std::cout<<"X: "<<solution[i]->as<mace::pose::CartesianPosition_2D>()->getXPosition()<<"Y: "<<solution[i]->as<mace::pose::CartesianPosition_2D>()->getYPosition()<<std::endl;
    }
//    mace::pose::CartesianPosition_2D* state1 = space.getNewState()->as<mace::pose::CartesianPosition_2D>();
//    //mace::pose::CartesianPosition_2D* cast = state1->
//    mace::pose::CartesianPosition_2D* state2 = space.copyState(state1)->as<mace::pose::CartesianPosition_2D>();
//    state2->setXPosition(50.0);
//    std::cout<<"Pause here"<<std::endl;
//    space.removeState(state1);
//    space.removeState(state2);
    std::cout<<"Pause here"<<std::endl;
//    NearestNeighbor_FLANN<mace::planners_sampling::RootNode*> tree =
//            NearestNeighbor_FLANN<mace::planners_sampling::RootNode*>(std::shared_ptr<flann::LinearIndexParams>(new flann::LinearIndexParams()));

//    Cartesian2DSpaceBounds bounds(-10,10,-10,10);
//    Cartesian2DSpace space;
//    space.setBounds(bounds);
//    Cartesian2DSpace_Sampler sampler(&space);
//    mace::pose::CartesianPosition_2D state;
//    sampler.sampleUniform(&state);

    //mace::nn::KDTreeTest test;

//    char* MACEPath = getenv("MACE_ROOT");
//    if(MACEPath){
//        std::string rootPath(MACEPath);
//        QFile inputFile(QString::fromStdString(rootPath + "/US48.txt"));
//        if (inputFile.open(QIODevice::ReadOnly))
//        {
//           QTextStream in(&inputFile);
//           while (!in.atEnd())
//           {

//              QString line = in.readLine();
//              QStringList list = line.split(" ");
//              std::string name = "point_" + std::to_string(counter);
//              double x = list.at(1).toDouble();
//              double y = list.at(2).toDouble();
//              mace::pose::Position<mace::pose::CartesianPosition_2D> point(name.c_str(), x, y);
//              vector.push_back(point);
//              counter++;
//           }
//           inputFile.close();
//        }
//    }


    return a.exec();
}
