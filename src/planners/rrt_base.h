#ifndef RRT_BASE_H
#define RRT_BASE_H

#include "nearest_neighbor_flann.h"
#include "base/pose/cartesian_position_2D.h"
#include "base/state_space/state.h"

namespace mace {
namespace nn{

class KDTreeTest
{
   public:
    KDTreeTest()
    {
        NearestNeighbor_FLANN<TestState*> tree = NearestNeighbor_FLANN<TestState*>(std::shared_ptr<flann::LinearIndexParams>(new flann::LinearIndexParams()));
        tree.setDistanceFunction([](const TestState* a, const TestState* b)
        {
            return std::fabs(a->getValue()-b->getValue());
        });

        TestState pos0;
        pos0.setValue(10.0);
        tree.add(&pos0);

        TestState pos1;
        pos1.setValue(20.0);
        tree.add(&pos1);

        TestState pos2;
        pos2.setValue(30.0);
        tree.add(&pos2);

        TestState* nearest = tree.nearest(&pos0);
        std::cout<<"I am complete"<<std::endl;
    }
};

}
}


//class Value
//{
//    Value():
//        value(0.0)
//    {

//    }

//    Value(const Value &copy)
//    {
//        this->value = copy.value;
//    }



//private:
//    double value = 0.0;
//};

//class RRT_Base
//{
//public:
//    RRT_Base();
//    flann::Index<flann::L2_Simple<double>> m_Tree;

//};

#endif // RRT_BASE_H
