#ifndef GRAPH_PLANNING_NODE_H
#define GRAPH_PLANNING_NODE_H

namespace mace {
namespace planners_graph{

class GraphNode
{
    GraphNode();

    ~GraphNode() = default;

public:
    double updateGValue(const double &newValue);

    double updateHValue(const double &newValue);

    void setParentNode(const GraphNode* parent);

    void hasBeenVisited(const bool &value);

    void hasBeenClosed(const bool &value);

public:
    double getFValue() const;

    double getGValue() const;

    double getHValue() const;

    bool isVisted() const;

    bool isClosed() const;

    GraphNode getParentNode() const;

private:
    void updateFValue();

private:
    double fValue;
    double gValue;
    double hValue;

    bool visited;
    bool closed;

    GraphNode* parentNode; /**< Member variable that contains a pointer to the parent node connecting
                             to this node currently offering the shortest known path. */
};


} //end of namespace planners_graph
} //end of namespace mace

#endif // GRAPH_PLANNING_NODE_H
