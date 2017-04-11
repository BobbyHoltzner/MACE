#pragma once
//#include<thread>
#include "ofMain.h"
#include"processThread.h"
#include"enviroment.h"



class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		ofTrueTypeFont myfont;
	private:
		bool updateFlag = false;
		bool lines = false;
		int agent = 5;
		float elapasedTime = -10.1;
		enviroment map;
		node *nodes;
		quadCopter *quad;
		//trajectoryPlanner vornoiThread;
		plannerThread *planner;
};
