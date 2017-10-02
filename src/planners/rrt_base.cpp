#include "rrt_base.h"

namespace mace {
namespace planners_sampling{

void RRTBase::setPlanningParameters(state_space::GoalState *begin, state_space::GoalState *end):
    m_stateBegin(begin), m_stateEnd(end)
{

}

//!
//! \brief RRTBase::solve function that initiates the solve routine of the planner
//!
void RRTBase::solve()
{

    /**
     * 1. Create the root node of the search based on the starting state and
     * insert into the allocated tree structure.
     */
    state_space::State* startState = m_spaceInfo->copyState(m_stateBegin);
    RootNode* start = new RootNode(startState);
    m_nnStrategy->add(start);

    {
        RootNode* sampleNode = new RootNode(m_spaceInfo->getStateSpace());
        //get the state from the node so that we can update this in memory when sampling
        state_space::State* sampleState = sampleNode->getCurrentState();

        /**
         * 2. Sample a state from the state space
         */
        if((m_stateEnd != nullptr) && (m_RNG.uniform01() < goalProbability))
            m_stateEnd->sampleGoal(sampleState);
        else
            m_samplingStrategy->sampleUniform(sampleState);

        /**
         * 3. Find the closet other node in the tree and determine the distance to the node
         */
        RootNode* closestNode = m_nnStrategy->nearest(sampleNode);
        state_space::State* closestState = closestNode->getCurrentState();
        double distance = m_spaceInfo->distanceBetween(closestState,sampleState);

        /**
         * 4. Determine if the sampled point is within the appropriate distance threshold.
         * If not, interpolate between the states if possible. If not, drop the sample and
         * move on. If so, continue.
         */
        if(distance > maxBranchLength)
        {
            //do the interpretation
            m_spaceInfo->getStateSpace()->
            //update the address of the
        }


    }
    //we need a start state and an end state

    //while termination conditions have not been met
    //state_space::GoalState goal;

    //check if the goal is not null, is it time to sample around the goal, is the goal region samplable
    //if( (goal!= nullptr) && (m_RNG.uniform01() < goalProbability))
    //sample the goal somehow
    //else
    //m_SamplingStrategy->sampleUniform();
}

void RRTBase::setGoalProbability(const double &probability)
{
    goalProbability = probability;
}

double RRTBase::getGoalProbability() const{
    return this->goalProbability;
}

void RRTBase::setMaxBranchLength(const double &length)
{
    maxBranchLength = length;
}

double RRTBase::getMaxBranchLength() const
{
    return this->maxBranchLength;
}

} //end of namespace planners_sampling
} //end of namespace mace
