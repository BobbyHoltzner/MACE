#include "a_star_base.h"

namespace mace {
namespace planners_graph{


std::vector<state_space::State*> AStarBase::solve()
{
    std::vector<state_space::State*> solutionVector;

    GraphNode startNode;
    startNode.setCurrentState(m_stateBegin->getState()->getClone());

    GraphNode endNode;
    endNode.setCurrentState(m_stateEnd->getState()->getClone());

    std::multiset<GraphNode> openSet;
    std::multiset<GraphNode>::iterator openSetIT;

    std::unordered_set<const state_space::State*> closedSet;
    openSet.insert(startNode);

    while(!openSet.empty())
    {
        openSetIT = openSet.begin();
        GraphNode currentNode = *openSetIT;
        ++openSetIT;
        for(;openSetIT != openSet.end(); openSetIT++)
        {
            GraphNode evalNode = *openSetIT;
            if((evalNode.getFValue() < currentNode.getFValue()) || ((evalNode.getFValue() == currentNode.getFValue()) &&
                                                                  evalNode.getHValue() < currentNode.getHValue()))
            {
                currentNode = evalNode;
            }
        }

        openSet.erase(currentNode);
        closedSet.insert(currentNode.getCurrentState());

        if(currentNode.getCurrentState() == m_stateEnd->getState())
            return solutionVector;

        //get the appropriate neighbors
        std::vector<state_space::State*> neighbors;
        for(int i = 0; i < neighbors.size(); i++)
        {
            GraphNode neighborNode;
            state_space::State* neighborState = neighbors.at(i);
            neighborNode.setCurrentState(neighborState);
            //An edge valid check ensures that the current state is valid, and the connection between states is valid
            if(!m_spaceInfo->isEdgeValid(neighborState,currentNode.getCurrentState()) || closedSet.count(neighborState))
                continue;

            //movement cost to go from the current node to the neighboring node
            double movementCost = m_spaceInfo->getTraversalCost(currentNode.getCurrentState(),neighborNode.getCurrentState());
            //the total movement cost is therefore equal to the previous nodes cost plus the cost to move
            double parentCost = currentNode.getGValue();
            double totalCost = parentCost + movementCost;

            //if the new path to neighbor is shorter || neighbor is not in OPEN
            if(totalCost < neighborNode.getGValue() || openSet.count(neighborNode) == 0)
            {
                neighborNode.updateGValue(parentCost,movementCost);
                neighborNode.updateHValue(m_spaceInfo->getTraversalCost(neighborNode.getCurrentState(),m_stateEnd->getState(),false));
                neighborNode.setParentNode(&currentNode);

                if(openSet.count(neighborNode) == 0)
                {
                    openSet.insert(neighborNode);
                }
            }
        }


    }
}

void AStarBase::setPlanningParameters(state_space::GoalState* begin, state_space::GoalState* end)
{
    m_stateBegin = begin;
    m_stateEnd = end;
}

} //end of namespace planners_graph
} //end of namespace mace
