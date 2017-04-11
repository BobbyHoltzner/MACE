#include "processThread.h"

void plannerThread::threadedFunction()
{
	this->lock();
	planning = true; planned = false;
	voronoi.logic(nodes);
	planned = true; planning = false;
	this->unlock();
}