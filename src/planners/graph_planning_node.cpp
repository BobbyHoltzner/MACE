#include "graph_planning_node.h"

namespace mace {
namespace planners_graph{
GraphNode::GraphNode()
{

}

GraphNode::~GraphNode()
{

}

double GraphNode::updateGValue(const double &newValue)
{
    this->gValue = newValue;
    this->updateFValue();
    return this->fValue;
}

double GraphNode::updateHValue(const double &newValue)
{
    this->hValue = newValue;
    this->updateFValue();
    return this->fValue;
}

void GraphNode::setParentNode(const GraphNode *parent)
{
    this->parentNode = parent;
}

void GraphNode::hasBeenVisited(const bool &value)
{
    this->visited = value;
}

void GraphNode::hasBeenClosed(const bool &value)
{
    this->closed = value;
}

void GraphNode::updateFValue()
{
    this->fValue = this->gValue + this->hValue;
}

double  GraphNode::getFValue() const
{
    return fValue;
}

double  GraphNode::getGValue() const
{
    return gValue;
}

double  GraphNode::getHValue() const
{
    return hValue;
}

bool  GraphNode::isVisted() const
{
    return visited;
}

bool  GraphNode::isClosed() const
{
    return closed;
}

GraphNode* GraphNode::getParentNode() const
{
    return this->parentNode;
}

} //end of namespace planners_graph
} //end of namespace mace
