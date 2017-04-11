#pragma once
#include"ofMain.h"
#include"Eigen/Dense"
using namespace Eigen;
//--------------------------------------------------------------Macros
#define M(e) ofMap(e,-1,1,0,1);
//--------------------------------------------------------------SimualationNode.h
#define stressCutOff 0.7
#define noiseProb 1
//--------------------------------------------------------------Enviroment.h
#define NODE_RADIUS 3
//--------------------------------------------------------------QuadCopter.h
#define radiusQuad 35
#define mVal 4
#define mForce 2
#define accur 0.9
//--------------------------------------------------------------TrajectoryPlanner.h
#define LogOddUnstress 0.9
#define LogOddStress 0.7
//--------------------------------------------------------------ofApp.h
#define TotalNodes 5000
#define AUTOMATIC
#define randomSeed 5
//#define CLK