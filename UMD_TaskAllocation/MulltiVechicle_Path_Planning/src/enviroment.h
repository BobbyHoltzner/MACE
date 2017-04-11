/**
*  @file    enviroment.h
*  @author  Ravi Bhdeshiya
*  @author  Derek Paley
*  @version 1.0
*
*  @brief UMD, Lab CDCL, A Path Planning Module For cooperative Swarm Optimization
*
*  @section DESCRIPTION
*
This file handle nodes rendering and updating process
*/
#pragma once
#include"simulationNode.h"
/**
*  @brief enviroment class
*/
class enviroment
{
public:
//--------------------------------------------------------------Function
	// Default constructor  
	enviroment() {};
	// Default destructor  
	~enviroment() {};
	// Setup method
	void setup(node *&nodes, unsigned int nodeCount);
	// Update method
	void update();
	// Render method draw nodes in enviroment.
	void render(const node *nodes);
	void renderGrid();
private:
//--------------------------------------------------------------Variables
	unsigned int nodeCount;
	bool grid = false;
};
/**
*   @brief  Setup the nodes and update its index
*
*   @param  nodes is object of node
*   @param  totalNodes is a unsigned int for define nodes
*   @return nothing
*/
inline void enviroment::setup(node *& nodes, unsigned int nodeCount)
{
	this->nodeCount = nodeCount;
	nodes = new node(nodeCount);
	nodes->computeField(nodeCount);
	std::sort(nodes->waypoint, nodes->waypoint + nodeCount, comapareByArtificialField); //also uncomment the struct operator function.
	for (int i = 0; i < nodeCount; i++)
	{
		nodes->waypoint[i].index = i;
		//nodes->waypoint[i].alive = (nodes->waypoint[i].artificialField <= 2e8) && (nodes->waypoint[i].artificialField >= 0) ? true : false;
		nodes->waypoint[i].priority = (nodes->waypoint[i].artificialField <= 2e8) && (nodes->waypoint[i].artificialField >= 0) ? true : false;
		//std::cout << "Node " << i<<" normlize "<< nodes->waypoint[i].location.norm() <<" index "<<nodes->waypoint[i].index<< std::endl;
		//std::cout << "Node " << i<<" Field:"<< nodes->waypoint[i].artificialField<<" Stress"<<nodes->waypoint[i].stressValue<< std::endl;
	}
}
/**
*   @brief  Update the nodes and update its index
*
*   @return nothing
*/
inline void enviroment::update()
{
}
/**
*   @brief  Render the nodes and update its index
*
*   @param  nodes is const object of node
*   @return nothing
*/
inline void enviroment::render(const node * nodes)
{
	ofFill();
	ofEnableAlphaBlending();
	for (int i = 0; i < nodeCount; i++)
	{
		int hue = nodes->waypoint[i].alive ? 80 : nodes->waypoint[i].logOddStress ? 130 : 20;
		ofSetColor(nodes->waypoint[i].color, hue);
		//ofDrawCircle(nodes->waypoint[i].x, nodes->waypoint[i].y, 2.5);
		if (nodes->waypoint[i].priority)
		{
			ofDrawCircle(nodes->waypoint[i].location.x(), nodes->waypoint[i].location.y(), NODE_RADIUS+2);
		}
		else
		{
			ofDrawCircle(nodes->waypoint[i].location.x(), nodes->waypoint[i].location.y(), NODE_RADIUS);
		}
		if (nodes->waypoint[i].logOddStress)
		{
			ofNoFill();
			ofSetColor(255, 0, 0); 
			ofDrawRectangle(nodes->waypoint[i].location.x() - (NODE_RADIUS+1), nodes->waypoint[i].location.y() - (NODE_RADIUS+1), 2*(NODE_RADIUS+1), 2 * (NODE_RADIUS + 1));
			ofFill();
		}
	}
	ofDisableAlphaBlending();
	ofNoFill();
	if (grid == true) renderGrid();
}
/**
*   @brief  Setup the nodes and update its index
*
*   @param  nothing
*   @return nothing
*/
inline void enviroment::renderGrid()
{
	ofEnableAlphaBlending();
	ofSetColor(20, 130, 0, 50);
	for (int i = 0; i < ofGetWindowWidth(); i += 5)
	{
		ofDrawLine(i, 0, i, ofGetWindowHeight());
	}
	for (int j = 0; j < ofGetWindowHeight(); j += 5)
	{
		ofDrawLine(0, j, ofGetWindowWidth(), j);
	}
	ofDisableAlphaBlending();
}