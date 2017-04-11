#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
#ifdef randomSeed
	ofSeedRandom(randomSeed);
#endif // randomSeed
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	ofSetWindowTitle("Simulation of collabrative path planning");
	ofBackground(255, 255, 255);
	myfont.loadFont("Roboto-Regular.ttf", 10);
	map.setup(nodes, TotalNodes);
	quad = new quadCopter[agent];
	planner = new plannerThread(nodes);
	//planner.vernoli.setup(quad, agent, nodes, totalNodes);//last

#ifdef randomSeed
	std::cout << "RandomSeed:" << randomSeed << endl;
#endif
	std::cout << endl << "toatalNodes: " << TotalNodes << endl;
	std::cout << "Actual StressNode: " << nodes->pStressCount << std::endl;
	std::cout << "Prior StressNode: " << nodes->stressCount << std::endl;
	std::cout << "Active agents: " << agent << " with accuracry: " << 100 * accur << "%" << std::endl;

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout<< std::endl << "Setup:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::update(){

	if (!updateFlag) return;
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // debug
	map.update();
#ifdef AUTOMATIC
	if (ofGetElapsedTimef() - elapasedTime > 10.0 && !planner->planning)
	{
		elapasedTime = ofGetElapsedTimef();
		planner->voronoi.setup(quad, agent, nodes, TotalNodes);
		planner->startThread();
		//std::thread planner (vornoiThread.setup, quad, agent, nodes, TotalNodes);
	}
#endif // AUTOMATIC

	if (planner->planned & !planner->planning)
	{
		//planner.unlock();
		planner->voronoi.updateForThread(quad);
	}


	for (int i = 0; i < agent; i++)
	{
		//Vector2f mouse(ofGetMouseX(), ofGetMouseY());
		//quad[i].controller(mouse);
		quad[i].fly(nodes);
		quad[i].update();
	}

	planner->voronoi.scanUpdate(quad, agent, nodes, TotalNodes);
#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::endl << "Update:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::draw(){
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	map.render(nodes);
	for (int i = 0; i < agent; i++)
	{
		quad[i].render();
		if (lines) quad[i].lines(nodes);
	}

	char fpsStr[255]; // an array of chars
	ofSetColor({ 255,0,0 });
	sprintf(fpsStr, "Frame rate: %f", ofGetFrameRate());
	myfont.drawString(fpsStr, ofGetWindowWidth() - 100, ofGetWindowHeight() - 10);
#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::endl << "Draw:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'q')
	{
		updateFlag = !updateFlag;
	}
	if (key == 'l')
	{
		lines = !lines;
	}
#ifndef AUTOMATIC
	if (key == 'p')
	{
		if (!planner.planning)
		{
			planner->voronoi.setup(quad, agent, nodes, totalNodes);
			planner->startThread();
			//planner.vernoli.updateForThread(quad);
		}
		//vernoli.setup(quad, agent, nodes, totalNodes);
		//vernoli.update(quad);
	}
	if (key == 'o')
	{
		//ofBaseApp::exit()
	}
#endif // !AUTOMATIC

	if (key == 'x') {
		ofImage img;
		img.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
		img.save("screenshot.png");
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
