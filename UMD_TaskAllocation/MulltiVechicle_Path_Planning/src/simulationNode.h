/**
*  @file    simulationNode.h
*  @author  Ravi Bhdeshiya
*  @author  Derek Paley
*  @version 1.0
*
*  @brief UMD, Lab CDCL, A Path Planning Module For cooperative Swarm Optimization
*
*  @section DESCRIPTION
*
This file genrate random nodes in simulation envrioment.
This nodes's properties are key to pathPlanning and partation.
*/
#pragma once
#include"simulationParam.h"
//#ifndef CLOCKNode
//#define CLOCKnode
//#endif // CLOCKNode
//--------------------------------------------------------------
struct waypoints
{
	int index;
	float stressValue;
	float logOdd = 0;
	float artificialField=0;
	Vector2f location;
	ofColor color;
	bool alive=true;
	bool logOddStress = false;
	bool stress;
	bool priority = false;
};

static bool compareByNorm(const waypoints& lhs, const waypoints& rhs)
{
	return lhs.location.norm() < rhs.location.norm();
}
static bool compareByStress(const waypoints& lhs, const waypoints& rhs)
{
	return lhs.stressValue < rhs.stressValue;
}
static bool comapareByArtificialField(const waypoints& lhs, const waypoints& rhs)
{
	return lhs.artificialField < rhs.artificialField;
}

//--------------------------------------------------------------
/**
*  @brief Node class for that genrate nodes and assigned stress value.
*/
class node
{
public:
//--------------------------------------------------------------Function
	// Default constructor  
	node() {};
	// Default Destructor  
	~node() {};
	// Main constructor  
	node(unsigned int totalNodes) { setup(totalNodes); }
	void setup(unsigned int totalNodes);
	void computeField(unsigned int totalNodes);
	float computeArtificialField(waypoints,waypoints);
	// Function to compute Stress 
	float computeStress(float,float);
	//--------------------------------------------------------------Variables
	waypoints *waypoint;///< Pointer:object of waypoint
	unsigned int pStressCount = 0;///< Int:Prior stress node counter
	unsigned int stressCount = 0;///Int:Stress node couonter after noise applied
};

inline void node::setup(unsigned int totalNodes)
{
#ifdef CLOCKnode
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG


	waypoint = new waypoints[totalNodes];

	for (int i = 0; i < totalNodes; i++)
	{
		float x = ofRandom(0, ofGetWindowWidth());
		float y = ofRandom(0, ofGetWindowHeight());
		waypoint[i].location << x, y;

		waypoint[i].stressValue = computeStress(x, y);
		waypoint[i].stress = waypoint[i].stressValue <= stressCutOff ? false : true;
		if (waypoint[i].stress) pStressCount++;

		float ran = M(ofRandomf()); //cout << ran << endl;
		waypoint[i].stress = waypoint[i].stress && ran > noiseProb ? !waypoint[i].stress : waypoint[i].stress;
		//if (waypoint[i].stress) stressCount++;

		if (waypoint[i].stress)
		{
			stressCount++;
			waypoint[i].color = { 255,40,40 };
		}
		else waypoint[i].color = { 10, 12, 160 };

		//std::cout << "Check:" <<i<<"::"<< waypoint[i].stress << "::" << waypoint[i].artificialField << std::endl;
		//TO plot......
		//std::cout << waypoint[i].location.x() << " " << waypoint[i].location.y() << " " << waypoint[i].stressValue << " " << waypoint[i].artificialField << std::endl;
	}


#ifdef CLOCKnode
	auto end = std::chrono::steady_clock::now();
	std::cout << "Nodes Genarted::" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}
inline void node::computeField(unsigned int totalNodes)
{
	for (int i = 0; i < totalNodes; i++)
	{
		for (int j = 0; j < totalNodes; j++)
		{
			if (i != j) 
			{
				waypoint[i].artificialField += computeArtificialField(waypoint[i], waypoint[j]);
			}
		}
		cout << "\r" << i;
	}
}
inline float node::computeArtificialField(const waypoints first,const waypoints second)
{
	Vector2f TempDist = first.location - second.location;
	if (TempDist.squaredNorm() < 5* radiusQuad) {
		float fieldValue = 9e9 * ((first.stress && second.stress) ? 1 : -1) / TempDist.squaredNorm();
		return fieldValue;
	}
	else return 0;
	
}
/**
*   @brief  ComputeStress method compute perlin noise based on different freq.
*
*   @param x is x cordinate of node
*   @param y is y cordinate of node
*   @return stress value in float
*/
inline float node::computeStress(float x, float y)
{
	x = (x / ofGetWindowWidth()) - 0.5; y = (y / ofGetWindowHeight()) - 0.5;
	return M(ofSignedNoise(5 * x, 5 * y)) + 0.5*M(ofSignedNoise(2 * x, 2 * y)) + 0.25*M(ofSignedNoise(4 * x, 4 * y));
}
