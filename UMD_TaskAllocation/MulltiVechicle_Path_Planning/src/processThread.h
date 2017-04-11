#pragma once
#include"trajectoryPlanner.h"

class plannerThread : public ofThread
{
public:
//--------------------------------------------------------------Function
	plannerThread(){
		planning = false;
		planned = false;
	}
	plannerThread(node *nodes) {
		planning = false;
		planned = false;
		this->nodes = nodes;
	}
	~plannerThread() {};
	//void setup(const trajectoryPlanner &p);
	void threadedFunction();
//--------------------------------------------------------------Variables
	bool planned, planning;
	node *nodes;
	trajectoryPlanner voronoi;
};
