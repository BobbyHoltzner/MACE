/**
*  @file    quadCopter.h
*  @author  Ravi Bhdeshiya
*  @author  Derek Paley
*  @version 1.0
*
*  @brief UMD, Lab CDCL, A Path Planning Module For cooperative Swarm Optimization
*
*  @section DESCRIPTION
*
This file genrate quadCopter in simualtion enviroment and handle all dynamic properties
*/
#pragma once
#include"simulationNode.h"
/**
*  @brief enviroment class
*/
class quadCopter
{
public:
//--------------------------------------------------------------Function
	// Default constructor  
	quadCopter() { setup(); }
	// Default destructor  
	~quadCopter() {};
	void setup();
	// Update method
	void update();
	//Render method
	void render();
	//Compute force addition
	void addForce(Vector2f force);
	//Controller genrate force toward target
	void controller(Vector2f target);
	//Find Path from assign node
	void fly(node *&nodes);
	//Render lines to connect the node if need 
	void lines(node *nodes);
	// Return state of quadCopter
	bool isAlive() { return alive; }
	// Return X cordinate
	float x() { return location.x(); }
	// Return Y cordinate
	float y() { return location.y(); }
	// Return scanning accuracy of quadCopter
	float accu() { return accuracy; }
	// Return scanning radius of quadCopter
	float getScanRadius() { return scanRadius; }
	// Return Location of quadCopter
	Vector2f getLocation() { return location; }
	//Return Color of quadCopter
	ofColor getColor() { return color; }

	waypoints findAlive(list<waypoints> &TRAJ,node *nodes);
//--------------------------------------------------------------Variables
	list<int> traj;
	list<waypoints> wTraj;
private:
	bool alive;
	float scanRadius,mass,accuracy;

	ofColor color;
	Vector2f HOME, location, velocity, accelaration, maxVelocity, maxForce;
	node *assignNodes;
	ofPolyline line;
	ofPoint pt;
};


inline void quadCopter::setup()
{
	alive = true; mass = 5.0; scanRadius = radiusQuad; accuracy = accur;
	//battery = 100;
	float x = ofRandom(0, ofGetWindowWidth()); float y = ofRandom(0, ofGetWindowHeight());

	location << x, y; HOME = location;
	velocity << 0.0, 0.0;
	accelaration << 0.0, 0.0;

	maxVelocity << mVal, mVal;
	maxForce << mForce, mForce;

	color = { ofRandom(0,255),ofRandom(0,255) ,ofRandom(0,255) };
}

inline void quadCopter::update()
{
	velocity += accelaration;
	velocity = (velocity.norm() <= maxVelocity.norm()) ? velocity : (velocity.normalized() *mVal);
	location += velocity;
	accelaration *= 0;

	pt.set(location.x(), location.y());
	line.addVertex(pt);
}

inline void quadCopter::render()
{
	int r = 6;
	ofEnableAlphaBlending();
	ofFill();
	ofSetColor(color);
	this->line.draw();
	//ofDrawCircle(location.x(), location.y(), r);
	ofNoFill();
	ofSetColor(color);
	ofDrawCircle(location.x(), location.y(), scanRadius);
	ofPushMatrix();
	ofTranslate(location.x(), location.y());
	//char Str[3]; // an array of chars
	////ofSetColor({ 0,0,0 });
	//sprintf(Str, "%d", battery);
	//myfont.drawString(Str, -10, scanRadius+2);
	ofRotate(ofRadToDeg(atan2(velocity.y(), velocity.x()) + PI / 2));
	ofFill();
	ofBeginShape();
	ofVertex(0, -r * 2);
	ofVertex(-r, r * 2);
	ofVertex(r, r * 2);
	ofEndShape(true);
	ofPopMatrix();
	//ofNoFill();
	//ofDrawCircle(location.x, location.y, scanRadius-3);
	ofNoFill();
	//for (float i = 0; i <= 0.1; i += 0.05) {
	//	ofDrawCircle(location.x(), location.y(), sin(i*ofGetFrameNum())*scanRadius);
	//	//ofDrawCircle(0, 0, sin(i*ofGetFrameNum())*scanRadius);
	//}
	//ofDrawBitmapString("quadA", ofGetMouseX(), ofGetMouseY() + 52);
	ofDisableAlphaBlending();
}

inline void quadCopter::addForce(Vector2f force)
{
	accelaration += (force / mass);
}

inline void quadCopter::controller(Vector2f target)
{
	Vector2f error = (target - location);
	//error.normalize();
	//error *= 1.5;
	//accelaration = error;
	Vector2f temp = error.normalized()*mVal;
	Vector2f steer = (temp - velocity);
	steer = (steer.norm() <= maxForce.norm()) ? steer : (steer.normalized() *mForce);
	addForce(steer);
}

inline void quadCopter::fly(node *& nodes)
{
	if (!wTraj.empty())
	{
		int index = 0;
		bool prio = false;
		Vector2f temp('inf', 'inf');
		for (auto i : wTraj)
		{
			if (nodes->waypoint[i.index].alive)
			{
				if (!nodes->waypoint[i.index].stress) nodes->waypoint[i.index].color = color;
				else nodes->waypoint[i.index].color = { 255,0,0 };
				Vector2f min = this->location - nodes->waypoint[i.index].location;
				if (min.norm()<temp.norm() && nodes->waypoint[i.index].priority)
				{
					temp = min;
					index = i.index;
					prio = true;
				}
			}
		}
		if (prio) {
			controller(nodes->waypoint[index].location);
		}
		//else if (!prio) {
		//	int index = 0;
		//	Vector2f temp('inf', 'inf');
		//	for (auto i : wTraj)
		//	{
		//		if (nodes->waypoint[i.index].alive)
		//		{
		//			if (!nodes->waypoint[i.index].stress) nodes->waypoint[i.index].color = color;
		//			else nodes->waypoint[i.index].color = { 255,0,0 };
		//			Vector2f min = this->location - nodes->waypoint[i.index].location;
		//			if (min.norm()<temp.norm() && nodes->waypoint[i.index].priority)
		//			{
		//				temp = min;
		//				index = i.index;
		//				prio = true;
		//			}
		//		}
		//	}
		//	controller(nodes->waypoint[index].location);
		//}

		//list<waypoints>::iterator it;
		//waypoints target = findAlive(wTraj,nodes);
		//controller(target.location);

	}
	else
	{
		//Vector2f mouse(ofGetMouseX(), ofGetMouseY());
		//controller(mouse);
		Vector2f HOME(0, 0);
		controller(HOME);
	}
}

inline void quadCopter::lines(node * nodes)
{
	ofEnableAlphaBlending();
	ofSetColor(this->color, 50);
	for (auto i : traj)
	{
		if (nodes->waypoint[i].alive)
		{
			ofDrawLine(x(), y(), nodes->waypoint[i].location.x(), nodes->waypoint[i].location.y());
		}
	}
	ofDisableAlphaBlending();
}

inline waypoints quadCopter::findAlive(list<waypoints> &TRAJ,node *nodes)
{
	list<waypoints>::iterator it;
	it = TRAJ.begin();
	if (TRAJ.size() <= 1)
	{
		return *it;
	}
	if (nodes->waypoint[it->index].alive)
	{
		return *it;
	}
	else
	{
		TRAJ.pop_front();
		return findAlive(TRAJ,nodes);
	}
}