/**
*  @file    trajectoryPlanning.h
*  @author  Ravi Bhdeshiya
*  @author  Derek Paley
*  @version 1.0
*
*  @brief UMD, Lab CDCL, A Path Planning Module For cooperative Swarm Optimization
*
*  @section DESCRIPTION
*
*  This is a part of framework that generate cooperative path plan
*  ,and allocate it to each UAV.Its take each UAV states
*  ,and active nodes states as input.Each vehicle�s distance from
*  the every node is computed.Each point is assigned to the
*  closest vehicle according to a Voronoi partition as output.
*/
#pragma once
#include"quadCopter.h"
#include<map>
class trajectoryPlanner
{
//--------------------------------------------------------------Function
public:
	// Default constructor  
	trajectoryPlanner();
	// Default Destructor  
	~trajectoryPlanner();
	// Takes input from all UAVs and Nodes
	void setup(quadCopter *quad, int activeAgent, node *nodes, int totalNode);
	// Updates waypoint for each UAV
	void update(quadCopter *&quad);
	void updateForThread(quadCopter *&quad);
	// Main logic function that compute Voronoi paration ,and similarly assign path to UAVs. 
	void logic();
	void logic(const node *nodes);
	// scan and update enviroment
	void scanUpdate(quadCopter *quad, int activeAgent, node *&nodes, int totalNode);
	bool scanStress(quadCopter quad, const waypoints waypoint);

private:
//--------------------------------------------------------------Variables
	const int X = 0, Y = 1;///< Constants for Readable Code
	int agent; ///< Size of active agent
	int nodeCount; ///< Size of active node in envrioment
	int count = 0;
	int stressCount = 0;
	MatrixXf euclidDist; ///< Data matrix for each UAV to each active nodes;Row=No of Active nodes,Cols:No of active agent + indexing.
						 //MatrixXd quadToWaypoint;
	list<int> *trajectory; ///< List for each UAV waypoints.

	list<waypoints> *traj;
						   //diff approch
	std::vector<std::pair<float, int>> *dist; ///< vector for each UAV waypoints.
	std::map<float, int> *distMap;///< Map based on dist=quad to node
	std::map<int, float> *indexMap;///< Map based on index
};