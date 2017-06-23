#include <environment_custom.h>

/**
 * @brief Environment_Map constructor
 * @param verts Vector of vertices that make up the environment boundary
 * @param gridSpacing Spacing between grid points
 */
Environment_Map::Environment_Map(const std::vector<Point> verts, double gridSpacing) :
    boundaryVerts(verts) {

    calculateBoundingRect(verts);
    initializeEnvironment(gridSpacing);
}

/**
 * @brief initializeEnvironment Initialize each node in the grid with a 0 value
 * @param gridSpacing Grid spacing
 * @return initial environment map
 */
std::map<double, std::map<double, Node> > Environment_Map::initializeEnvironment(const double gridSpacing) {
    std::map<double, std::map<double, Node> > tmpNodes;
    // Set up X points:
    int numXPoints = (int)std::floor((boundingRect.max.x - boundingRect.min.x)/gridSpacing);
    std::vector<double> xVals = createRange(boundingRect.min.x, boundingRect.max.x, numXPoints);
    // Set up Y points:
    int numYPoints = (int)std::floor((boundingRect.max.y - boundingRect.min.y)/gridSpacing);
    std::vector<double> yVals = createRange(boundingRect.min.y, boundingRect.max.y, numYPoints);

    for(auto xVal : xVals) {
        std::map<double, Node> y_node_map_tmp;
        for(auto yVal : yVals) {
            Point tmpPoint(xVal, yVal);
            Node tmpNode(tmpPoint, 0.0);
            y_node_map_tmp.insert(std::make_pair(yVal, tmpNode));

//                std::cout << "X Val: " << xVal << " / Y Val: " << yVal << std::endl;
        }

        tmpNodes.insert(std::make_pair(xVal, y_node_map_tmp));
    }

    // Set environment nodes and return created nodes:
    nodes = tmpNodes;
    return nodes;
}

/**
 * @brief setBoundaryVerts Set the new boundary vertices
 * @param verts Vector of Points defining the environment boundary
 */
void Environment_Map::setBoundaryVerts(std::vector<Point> verts) {
    boundaryVerts = verts;
    calculateBoundingRect(verts);
}

/**
 * @brief setNodeValue Set the value of the node closest to the given (x,y) point
 * @param location (x,y) point corresponding to the node we want to set
 * @param value New value for node
 * @return Success/failure
 */
bool Environment_Map::setNodeValue(const Point location, double value) {
    Point closestPt;
    bool foundPoint = findClosestPoint(location, closestPt);

    if(foundPoint) {
        nodes.at(closestPt.x).at(closestPt.y).value = value;
    }

    return foundPoint;
}

/**
 * @brief getNodeValue Get the value of the node closest to the given (x,y) point
 * @param location (x,y) point corresponding to the node we want to set
 * @param node Node closest to the given point
 * @return Success/failure
 */
bool Environment_Map::getNodeValue(const Point location, Node &node) {
    bool foundNode = findClosestNode(location, node);

    if(foundNode){
        std::cout << "Closest node at (" << node.location.x << ", " << node.location.y << "), value: " << node.value << std::endl;
    }

    return foundNode;
}

/**
 * @brief getNodesInPolygon Get the nodes contained in the polygon provided
 * @param boundary Vector of points that make up the footprint we want to check
 * @return Map of nodes within the footprint
 */
std::map<double, std::map<double, Node> > Environment_Map::getNodesInPolygon(std::vector<Point> boundary) {
    std::map<double, std::map<double, Node> > containedNodes;
    int count = 0;
    for(auto nodeX : nodes) {
        double xVal = nodeX.first;

        std::map<double, Node> y_node_map_tmp;
        for(auto nodeY : nodeX.second) {
            double yVal = nodeY.first;

            Point tmpPoint = Point(xVal, yVal);
            bool inPoly = pointInPoly(boundary, tmpPoint);

            if(inPoly){
                count ++;
                y_node_map_tmp.insert(std::make_pair(yVal, nodeY.second));
            }
        }

        if(y_node_map_tmp.size() > 0) {
            containedNodes.insert(std::make_pair(xVal, y_node_map_tmp));
        }
    }


    std::cout << "Number of nodes in footprint: " << containedNodes.size() << " / Total nodes: " << nodes.size() << std::endl;

    return containedNodes;
}

/**
 * @brief findClosestPoint Find the closest (x,y) grid point corresponding to the (x,y) pair given
 * @param testPoint Location we want to grab the corresponding grid point for
 * @param closestPoint Grid point closest to the provided point
 * @return Success/failure
 */
bool Environment_Map::findClosestPoint(Point testPoint, Point &closestPoint) {

    // First check if the point is even in the environment we are observing:
    if(pointInPoly(boundaryVerts, testPoint)) {
        auto xPoint = nodes.lower_bound(testPoint.x);
        if(xPoint != nodes.end()) {
            auto xPrev = nodes.lower_bound(testPoint.x);
            if(xPrev != nodes.begin()) {
                --xPrev;
            }

            if(fabs(xPrev->first-testPoint.x) < fabs(xPoint->first-testPoint.x)){
                xPoint = xPrev;
            }


            auto yPoint = xPoint->second.lower_bound(testPoint.y);
            if(yPoint != xPoint->second.end()) {
                auto yPrev = xPoint->second.lower_bound(testPoint.y);
                if(yPrev != xPoint->second.begin()) {
                    --yPrev;
                }

                if(fabs(yPrev->first-testPoint.y) < fabs(yPoint->first-testPoint.y)){
                    yPoint = yPrev;
                }

                closestPoint = Point(xPoint->first, yPoint->first);
            }

            return true;
        }
        else {
            std::cout << "No point close to (" << testPoint.x << ", " << testPoint.y << std::endl;
            closestPoint = Point(NAN, NAN); // TODO: Figure out better way to denote "not found"

            return false;
        }
    }
    else {
        std::cout << "(" << testPoint.x << ", " << testPoint.y << ") is outside the environment we are observing." << std::endl;
        closestPoint = Point(NAN, NAN);

        return false;
    }
}

/**
 * @brief distanceBetweenPoints Caluclate the distance between two points
 * @param pt1
 * @param pt2
 * @return Distance
 */
double Environment_Map::distanceBetweenPoints(Point pt1, Point pt2) {
    double xDiff = pt2.x - pt1.x;
    double yDiff = pt2.y - pt1.y;
    return sqrt((xDiff*xDiff) + (yDiff*yDiff));
}

/**
 * @brief findClosestNode Get the node at the closest point to the (x,y) pair provided
 * @param testPoint (x,y) pair we want to find the corresponding grid point for
 * @param closestNode Node at the closest grid point
 * @return Succes/failure
 */
bool Environment_Map::findClosestNode(Point testPoint, Node &closestNode) {
    Point closestPt;
    bool foundPt = findClosestPoint(testPoint, closestPt);

    if(foundPt){
        closestNode = nodes.at(closestPt.x).at(closestPt.y);
    }
    return foundPt;
}

/**
 * @brief calculateBoundingRect Calculate the bounding rectangle given a set of vertices
 * @param verts Vertices to calculate the bounding rectangle for
 */
void Environment_Map::calculateBoundingRect(const std::vector<Point> verts) {
    BoundingRect rect;
    rect.min = Point(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    rect.max = Point(-1, -1);

    double maxXVal = rect.max.x;
    double minXVal = rect.min.x;
    double maxYVal = rect.max.y;
    double minYVal = rect.min.y;

    for(auto val : verts) {
        // Check for min/max X
        if(val.x > maxXVal) maxXVal = val.x;
        if(val.x < minXVal) minXVal = val.x;
        // Check for min/max Y
        if(val.y > maxYVal) maxYVal = val.y;
        if(val.y < minYVal) minYVal = val.y;
    }

    // Set our new bounding rectangle:
    boundingRect.min = Point(minXVal, minYVal);
    boundingRect.max = Point(maxXVal, maxYVal);
}

/**
 * @brief createRange Create a vector of evenly space numbers
 * @param min Minimum value
 * @param max Maximum value
 * @param N Number of values in the range
 * @return Vector of evenly spaced values from min to max
 */
std::vector<double> Environment_Map::createRange(double min, double max, int N) {
    std::vector<double> range;
    double delta = (max-min)/double(N);
    for(int i=0; i<=N; i++) {
        range.push_back(min + i*delta);
    }
    return range;
}


// method taken from: https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html and converted to my notation
/**
 * @brief pointInPoly Determine if a point is in a polygon
 * @param vertices Vertices of a polygon to check
 * @param testPoint Point to check if in the polygon
 * @return True if inside the polygon, false if outside the polygon
 */
bool Environment_Map::pointInPoly(std::vector<Point> vertices, Point testPoint)
{
    int nvert = boundaryVerts.size();

    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if (
                ((vertices[i].y > testPoint.y) != (vertices[j].y > testPoint.y)) &&
                (testPoint.x < (vertices[j].x-vertices[i].x) * (testPoint.y-vertices[i].y) / (vertices[j].y-vertices[i].y) + vertices[i].x)
             ) {
                // If we've crossed a boundary, toggle the counter
                c = !c;
        }
    }

    // If c, which denotes the number of times a horizontal ray has crossed a boundary, is 0, that means we
    //        are outside of the polygon (we've crossed an even number of times). Else, we are inside the polygon
    if(c == 0) {
      return false;
    }
    else {
      return true;
    }
}
