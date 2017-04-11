#include "TrajectoryPlanner.h"
/**
*   @brief  Default  constructor for trajectoryPlanner
*
*   @param  void
*   @return nothing
*/
trajectoryPlanner::trajectoryPlanner()
{
}
/**
*   @brief  Default  Destructor for trajectoryPlanner
*
*   @param  void
*   @return nothing
*/
trajectoryPlanner::~trajectoryPlanner()
{

}
/**
*   @brief  Setup method for trajectoryPlanner
*
*   @param  quad is a array pointer variable to quadcopters objects
*   @param  activeAgent is an integer variable for No of active agent
*   @param  nodes is a array pointer to node objects
*   @param  totalNode is an integer variable for No of active nodes
*
*   @return nothing
*/
void trajectoryPlanner::setup(quadCopter * quad, int activeAgent, node * nodes, int totalNode)
{
#ifdef DEBUG
	cout << endl << "Debug Mode Active:" << endl;
#endif // DEBUG
	agent = activeAgent;
	nodeCount = 1;
	//>>>>>
	//dist = new vector<std::pair<float, int>>[agent];
	distMap = new map<float, int>[agent];
	indexMap = new map<int, float>[agent];
	//>>>>>
	for (int i = 0; i < totalNode; i++)
	{
		if (nodes->waypoint[i].alive)
		{
			nodeCount++;
			for (int j = 0; j < agent; j++)
			{
				// Distance Calculation
				Vector2f TempDist = nodes->waypoint[i].location - quad[j].getLocation();
				//>>>>>
				//dist[j].push_back(std::pair<float, int>(TempDist.norm(), nodes->waypoint[i].index));
				distMap[j][TempDist.norm()] = nodes->waypoint[i].index;
				indexMap[j][nodes->waypoint[i].index] = TempDist.norm();
				//>>>>>
			}
		}
	}
#ifdef DEBUG
	for (size_t j = 0; j < agent; j++)
	{
		cout << "Agent " << j << endl;
		for (std::map<float, int>::iterator i = distMap[j].begin(); i != distMap[j].end(); i++)
		{
			//for (std::map<int, float>::iterator k = indexMap[j].begin(); k != indexMap[j].end(); k++)
			//{
			cout << i->first << "  " << i->second << "  " << endl;// k->first << "  " << k->second << endl;
																  //}
		}
	}
#endif // DEBUG
}
/**
*   @brief  Setup method for trajectoryPlanner
*
*   @param  quad is a array pointer variable to quadcopters objects
*
*   @return nothing
*/
void trajectoryPlanner::update(quadCopter *&quad)
{
	//Main logic funtion
#ifdef CLOCK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	logic();
#ifdef CLOCK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG

	for (int i = 0; i < agent; i++)
	{
		//Copy trajectory to quad
		quad[i].traj.assign(trajectory[i].begin(), trajectory[i].end());//copy assign;
#ifdef DEBUG
		cout << "Quad " << i << endl << "Assign" << endl;
		for (auto v : trajectory[i]) {
			std::cout << v << "\n";
		}
#endif
	}
}
void trajectoryPlanner::updateForThread(quadCopter *& quad)
{
	for (int i = 0; i < agent; i++)
	{
		//Copy trajectory to quad
		//quad[i].traj.assign(trajectory[i].begin(), trajectory[i].end());//copy assign;
		//traj[i].sort(comapareByArtificialField);
		quad[i].wTraj = traj[i];
	}
}
/**
*   @brief  logic method for trajectoryPlanner
*
*   @param  void
*
*   @return nothing
*/
void trajectoryPlanner::logic()
{
	//init new tarj list
	trajectory = new list<int>[agent];
	int limit = nodeCount/agent;
	//std::cout << std::endl << "Limit:" << limit << std::endl;
	//gredy assignment
	
	
	bool flag = true;
	while (flag)
	{
		int index;
		float min;
		//get avilable vehicle if not check flag to break loop
		for (int i = 0; i < agent; i++)
		{
			if (!distMap[i].empty())
			{
				index = i;
				min = distMap[index].begin()->first;
				flag = true;
				break;
			}
			else
			{
				flag = false;
			}
		}
		if (!flag) break;
		//find min value for first key
		for (int i = 0; i < agent; i++)
		{
			if (!distMap[i].empty())
			{
				if (distMap[i].begin()->first < min)
				{
					min = distMap[i].begin()->first;
					index = i;
				}
			}
		}
		//if list empt then go for assignment and remove key from every agent
		//std::cout << "check:" << trajectory[index].size() << " " << limit << " and " << (trajectory[index].size() < limit) << endl;
		if (trajectory[index].size() <1.5*limit)
		{
			int waypointIndex = distMap[index].begin()->second;
			trajectory[index].push_back(waypointIndex);
			//#ifdef DEBUG
			//cout << "Agent:" << index << " : " << "Waypoint:" << waypointIndex << endl;
			//#endif // DEBUG
			for (int i = 0; i < agent; i++)
			{
				float key = indexMap[i].at(waypointIndex);
				indexMap[i].erase(waypointIndex);
				distMap[i].erase(key);
			}
		}
		//if not then remove key form this vehicle
		else
		{
			distMap[index].erase(distMap[index].begin());
		}
	}
}
void trajectoryPlanner::logic(const node *nodes)
{
	//init new tarj list
	//trajectory = new list<int>[agent];
	traj = new list<waypoints>[agent];
	int limit = nodeCount / agent;
	//std::cout << std::endl << "Limit:" << limit << std::endl;
	//gredy assignment


	bool flag = true;
	while (flag)
	{
		int index;
		float min;
		//get avilable vehicle if not check flag to break loop
		for (int i = 0; i < agent; i++)
		{
			if (!distMap[i].empty())
			{
				index = i;
				min = distMap[index].begin()->first;
				flag = true;
				break;
			}
			else
			{
				flag = false;
			}
		}
		if (!flag) break;
		//find min value for first key
		for (int i = 0; i < agent; i++)
		{
			if (!distMap[i].empty())
			{
				if (distMap[i].begin()->first < min)
				{
					min = distMap[i].begin()->first;
					index = i;
				}
			}
		}
		//if list empt then go for assignment and remove key from every agent
		//std::cout << "check:" << trajectory[index].size() << " " << limit << " and " << (trajectory[index].size() < limit) << endl;
		if (traj[index].size() <1.5*limit)
		{
			int waypointIndex = distMap[index].begin()->second;
			//trajectory[index].push_back(waypointIndex);
			////////////////////////////////////////////////////////////////////////////////////////////////<<<
			traj[index].push_back(nodes->waypoint[waypointIndex]);

			//#ifdef DEBUG
			//cout << "Agent:" << index << " : " << "Waypoint:" << waypointIndex << endl;
			//#endif // DEBUG
			for (int i = 0; i < agent; i++)
			{
				float key = indexMap[i].at(waypointIndex);
				indexMap[i].erase(waypointIndex);
				distMap[i].erase(key);
			}
		}
		//if not then remove key form this vehicle
		else
		{
			distMap[index].erase(distMap[index].begin());
		}
	}
}

void trajectoryPlanner::scanUpdate(quadCopter * quad, int activeAgent, node *&nodes, int totalNodes)
{
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	for (int j = 0; j < agent; j++)
	{
		for (auto i : quad[j].wTraj)
		{
			if (nodes->waypoint[i.index].alive)
			{
				Vector2f dist = quad[j].getLocation() - nodes->waypoint[i.index].location;
				if (dist.norm() <= quad[j].getScanRadius())
				{
					nodes->waypoint[i.index].logOdd += scanStress(quad[j], nodes->waypoint[i.index]) ? LogOddStress : -1 * LogOddUnstress;

					if (nodes->waypoint[i.index].logOdd <= -1 * LogOddUnstress || nodes->waypoint[i.index].logOdd >= 4 * LogOddStress)
					{
						nodes->waypoint[i.index].alive = false;
						if (nodes->waypoint[i.index].logOdd <= -1 * LogOddUnstress)
						{
							nodes->waypoint[i.index].stress = false;
							count++; //nodes->waypoint[i].color = quad[j].getColor();
						}
						else if (nodes->waypoint[i.index].logOdd >= 4 * LogOddStress)
						{
							nodes->waypoint[i.index].logOddStress = true;
							//cout << "logOdd:" << nodes->waypoint[i].logOdd << endl;
							stressCount++; //nodes->waypoint[i].color = { 255,0,0 };
						}
					}
					cout << "\r" << stressCount << " stress node and " << stressCount + count << " total nodes are found.";
				}

			}
		}
	}
#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << "ScanUpdate:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

bool trajectoryPlanner::scanStress(quadCopter quad, const waypoints waypoint)
{
	bool result = (waypoint.stressValue >= stressCutOff) ? true : false;
	float rand = ofMap(ofRandomf(), -1, 1, 0, 1);
	return result = (rand <= quad.accu()) ? result : !result;
}
