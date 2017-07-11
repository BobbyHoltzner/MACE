#include <environment_custom.h>

#include <algorithm>

using namespace voro;

// This function returns a random double between 0 and 1
double rnd() {return double(rand())/RAND_MAX;}

/**
 * @brief Environment_Map constructor
 * @param verts Vector of vertices that make up the environment boundary
 * @param gridSpacing Spacing between grid points
 */
Environment_Map::Environment_Map(const std::vector<Point> verts, double gridSpacing) :
    boundaryVerts(verts) {

    calculateBoundingRect(verts);
    initializeEnvironment(gridSpacing);

    // Testing:
//    std::map<int, Point> vehicleCells;
//    vehicleCells.insert(std::make_pair(5, Point(-2.5,-2.5,0)));
//    vehicleCells.insert(std::make_pair(2, Point(2.5,2.5,0)));
//    computeVoronoi(boundingRect, vehicleCells);
    // End testing
}

/**
 * @brief computeVoronoi Given the bounding box and current vehicle positions, compute a voronoi diagram
 * @param bbox Bounding box
 * @param vehicles Positions of vehicles (in x,y,z coordinates)
 */
void Environment_Map::computeVoronoi(const BoundingBox bbox, const std::map<int, Point> vehicles, GridDirection direction) {
    // Set up constants for the container geometry
    const double x_min = bbox.min.x, x_max = bbox.max.x;
    const double y_min = bbox.min.y, y_max = bbox.max.y;
    const double z_min = -0.5, z_max = 0.5;

    double x,y,z;
    voronoicell_neighbor c;

    // Set up the number of blocks that the container is divided into
    const int n_x=1, n_y=1, n_z=1;

    // Create a container with the geometry given above, and make it
    // non-periodic in each of the three coordinates. Allocate space for
    // eight particles within each computational block
    container con(x_min,x_max,y_min,y_max,z_min,z_max,n_x,n_y,n_z,
                 false,false,false,8);

    // Import into container
    //con.import("pack_six_cube");
    // Add vehicles/particles into the container only if they are in the environment
    for(auto vehicle : vehicles) {
        if(pointInPoly(boundaryVerts, vehicle.second)) {
            con.put(vehicle.first, vehicle.second.x, vehicle.second.y, 0);
        }
        else {
            std::cout << "Vehicle at position (" << vehicle.second.x << ", " << vehicle.second.y << ") not within environment. Skipping." << std::endl;
        }
    }

    // Loop over all vehicles in the container and compute each Voronoi
    // cell
    c_loop_all cl(con);
    int dimension = 0;
    if(cl.start()) do if(con.compute_cell(c,cl)) {
        dimension+=1;
    } while (cl.inc());

    // Initialze containers:
    std::vector<std::vector<int> > face_connections(dimension);
    std::vector<std::vector<double> > vertex_positions(dimension);

    int counter = 0;
    if(cl.start()) do if(con.compute_cell(c,cl)) {
        // Grab cell position:
        cl.pos(x,y,z);

        // Initialize vectors for vertex index and points:
        std::vector<int> f_vert;
        std::vector<double> v;

        // Grab vertex indeces and points
        c.face_vertices(f_vert);
        c.vertices(x,y,z,v);

        // Add to vector containers:
        face_connections[counter] = f_vert;
        vertex_positions[counter] = v;

        // Initialize Cell variables:
        int count = 1;
        double xVal = NAN;
        double yVal = NAN;
        std::vector<Point> coords;
        // Loop over vertex positions and determine which is the x coordinate, and which is y coordinate:
        for(auto vertexCoord : vertex_positions[counter]) {
            if(count == 1 ) {
                // X coord:
                xVal = vertexCoord;
                count++;
            }
            else if(count == 2) {
                // Y coord:
                yVal = vertexCoord;
                count++;
            }
            else {
                // Z coord: add our point, reset counter
                // Add point to our vector:
                if(vertexCoord < 0) {
                    coords.push_back(Point(xVal, yVal, 0));
                }
                count = 1;
            }
        }

        // Increment counter:
        counter += 1;

        // Create cell and add to map:
        Cell cell;
        cell.site.x = x;
        cell.site.y = y;
        cell.site.z = z;
        cell.vertices = coords;
        // Sort the cell vertices:
        sortCellVerticesCCW(cell);
        // Get contained nodes:
        setNodesInCell(cell);

        // TODO: There's got to be a better way to sort the North/South direction then creating a whole separate map of y-vals:
        setContainedNodesYX(cell);

        // Sort the nodes:
        sortNodesInGrid(cell, direction);
        // Get vehicle ID:
        int vehicleID = getVehicleID(cell, vehicles);
        // Insert into our map:
        //  - If our vehicle ID is < 0 (i.e. -1), we didn't find a vehicle in our cell. Something went wrong
        if(vehicleID > 0){
            cells[vehicleID] = cell;
        }
      } while (cl.inc()); // Increment to the next cell:


//      // Print cell sites and vertices:
//      for(auto cell : cells) {
//        std::cout << "\n\Vehicle ID " << cell.first << " site: (" << cell.second.site.x << ", " << cell.second.site.y << ", " << cell.second.site.z << ")" << std::endl;
//        int vertCount = 1;
//        for(auto cellVert : cell.second.vertices) {
//            double xTmp = cellVert.x;
//            double yTmp = cellVert.y;
//            double zTmp = cellVert.z;
//            std::cout << "Cell vertex " << vertCount << ": (" << xTmp << ", " << yTmp << ", " << zTmp << ")" << std::endl;
//            vertCount++;
//        }
//      }

      // Output the particle positions in gnuplot format
      con.draw_particles("polygons_p.gnu");

      // Output the Voronoi cells in gnuplot format
      con.draw_cells_gnuplot("polygons_v.gnu");
}

/**
 * @brief getVehicleID Check a cell for the vehicle contained within the cell to grab its ID
 * @param cell Cell who's boundary we will check
 * @param vehicleList List of vehicles and their positions to check
 * @return Vehicle ID
 */
int Environment_Map::getVehicleID(const Cell cell, const std::map<int, Point> vehicleList) {
    int vehicleID = -1;
    // For every vehicle in the vehicle list, test if the vehicle position is within the cell boundary
    //  -If we have multiple vehicles within the cell boundary, check which one is closest to the cell's site position
    //      - Whichever is closer, grab that ID
    //  -If we only have one vehicle, then treat that vehicle as the vehicle as the cell's vehicle and grab its ID
    //  -IF we don't get any vehicle within the boundary, then its probably time for a replan. return a -1

    double prevDist = std::numeric_limits<double>::max();
    for(auto vehicle : vehicleList) {
        bool inCell = pointInPoly(cell.vertices, vehicle.second);
        if(inCell) {
            if(vehicleID < 0) {
                // Means we don't have a vehicle yet. Set our vehicle ID to this vehicle ID
                vehicleID = vehicle.first;
                prevDist = distanceBetweenPoints(cell.site, vehicle.second);
            }
            else {
                // Means we already have a vehicle ID. Check which position is closer--that is what we will treat as our cell's vehicle
                double dist = distanceBetweenPoints(cell.site, vehicle.second);
                if(dist < prevDist) {
                    vehicleID = vehicle.first;
                }
            }
        }
    }

    return vehicleID;
}

/**
 * @brief sortCellVertices Sort the vertices of a cell in CCW fashion
 * @param cell Cell to update vertex ordering
 */
void Environment_Map::sortCellVerticesCCW(Cell &cell) {
    // 1) Create empty cell vertices vector
    // 2) Calculate polar angles and put into vector
    // 3) Find smallest angle
    // 4) Use index to add to cell vertices vector
    // 5) Remove smallest angle, find new smallest angle
    // 6) Use index to add to cell vertices vector

    //  1) Calculate angle between site and all vertices
    std::vector<double> angles;
    std::vector<Point> sortedVerts;
    for(auto cellVert : cell.vertices) {
        double angle = atan2(cellVert.y - cell.site.y, cellVert.x - cell.site.x) * (180/M_PI);
        angles.push_back(angle);
    }

    while(sortedVerts.size() < cell.vertices.size()) {
        std::vector<double>::iterator result = std::min_element(std::begin(angles), std::end(angles));
        int index = std::distance(std::begin(angles), result);
        // Add vertex at this index to our new vector:
        sortedVerts.push_back(cell.vertices.at(index));

        // Set angle to MAX double value:
        angles.at(index) = std::numeric_limits<double>::max();
    }

    // Set our vertices to sorted vertices:
    cell.vertices = sortedVerts;
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
            Point tmpPoint(xVal, yVal, 0);
            Node tmpNode(tmpPoint, 0.0);
            y_node_map_tmp.insert(std::make_pair(yVal, tmpNode));
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
 * @brief setNodesInCell Get the nodes contained in the polygon provided in a cell
 * @param Cell Cell with boundary of points that make up the footprint we want to check
 */
void Environment_Map::setNodesInCell(Cell &cell) {
    // Clear unsorted points:
    cell.unsortedContainedPoints.clear();

    std::map<double, std::map<double, Node> > tmpContainedNodes;
    int count = 0;
    for(auto nodeX : nodes) {
        double xVal = nodeX.first;

        std::map<double, Node> y_node_map_tmp;
        for(auto nodeY : nodeX.second) {
            double yVal = nodeY.first;

            Point tmpPoint = Point(xVal, yVal,0);
            bool inPoly = pointInPoly(cell.vertices, tmpPoint);

            if(inPoly){
                count ++;
                y_node_map_tmp.insert(std::make_pair(yVal, nodeY.second));

                // Insert into unsorted points vector:
                cell.unsortedContainedPoints.push_back(Point(xVal, yVal, 0.0));
            }
        }

        if(y_node_map_tmp.size() > 0) {
            tmpContainedNodes.insert(std::make_pair(xVal, y_node_map_tmp));
        }
    }


//    std::cout << "Number of nodes in footprint: " << containedNodes.size() * containedNodes.begin()->second.size() << " / Total nodes: " << nodes.size() * nodes.begin()->second.size() << std::endl;

    // Set containedNodes
    cell.containedNodes = tmpContainedNodes;
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

                closestPoint = Point(xPoint->first, yPoint->first, 0);
            }

            return true;
        }
        else {
            std::cout << "No point close to (" << testPoint.x << ", " << testPoint.y << std::endl;
            closestPoint = Point(NAN, NAN, NAN); // TODO: Figure out better way to denote "not found"

            return false;
        }
    }
    else {
        std::cout << "(" << testPoint.x << ", " << testPoint.y << ") is outside the environment we are observing." << std::endl;
        closestPoint = Point(NAN, NAN, NAN);

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
    BoundingBox rect;
    rect.min = Point(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    rect.max = Point(-1, -1, -1);

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
    boundingRect.min = Point(minXVal, minYVal, 0);
    boundingRect.max = Point(maxXVal, maxYVal, 0);
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
    // First check if the point lies on any of the line segments created by the boundary. If so, return true:
    int vertCount = 0;
    for(auto vert : vertices) {
        Point vert1, vert2;
        if(vertCount == 0){
            vert1 = vert;
            vert2 = vertices[vertices.size()-1];
        }
        else {
            vert1 = vertices[vertCount-1];
            vert2 = vert;
        }
        double epsilon = 0.00001;
        if(distanceToSegment(vert1, vert2, testPoint) < epsilon)
            return true;

        // Increment counter
        vertCount++;
    }

    // If not on any of the line segments, check if inside the polygon:
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

/**
 * @brief distanceToSegment Determine shortest distance to a line segment
 * @param p1 First vertex of the line segment
 * @param p2 Second vertex of the line segment
 * @param testPoint Point to check
 * @return Distance to the line segment
 */
double Environment_Map::distanceToSegment(Point p1, Point p2, Point testPoint) {
    double diffX = p2.x - p1.x;
    float diffY = p2.y - p1.y;
    if ((diffX == 0) && (diffY == 0))
    {
        diffX = testPoint.x - p1.x;
        diffY = testPoint.y - p1.y;
        return sqrt(diffX * diffX + diffY * diffY);
    }

    float t = ((testPoint.x - p1.x) * diffX + (testPoint.y - p1.y) * diffY) / (diffX * diffX + diffY * diffY);

    if (t < 0)
    {
        //point is nearest to the first point i.e p1.x and p1.y
        diffX = testPoint.x - p1.x;
        diffY = testPoint.y - p1.y;
    }
    else if (t > 1)
    {
        //point is nearest to the end point i.e p2.x and p2.y
        diffX = testPoint.x - p2.x;
        diffY = testPoint.y - p2.y;
    }
    else
    {
        //if perpendicular line intersect the line segment.
        diffX = testPoint.x - (p1.x + t * diffX);
        diffY = testPoint.y - (p1.y + t * diffY);
    }

    //returning shortest distance
    return sqrt(diffX * diffX + diffY * diffY);
}


/**
 * @brief addVehicle Update/insert a vehicle in our map and re-compute the voronoi partition
 * @param vehicleID ID of the vehicle to add
 * @param position Last known position of the vehicle
 * @return True if we should send to MACE core, false if not
 */
bool Environment_Map::updateVehiclePosition(const int vehicleID, const Point position, bool recomputeVoronoi) {
    // Add/overwrite our vehicle position to the map
    vehicles[vehicleID] = position;

    // If recompute flag is set, recompute the voronoi partition:
    if(recomputeVoronoi) {
        computeVoronoi(boundingRect, vehicles, GridDirection::NORTH_SOUTH);
        return true; // Send to MACE Core
    }

    return false; // Don't send to MACE core
}

/**
 * @brief sortNodesInGridSort the nodes in the cell in a grid fashion
 * @param cell Cell to sort the nodes
 * @param direction Direction to sort nodes (north/south, east/west, or by closest node)
 */
std::vector<Point> Environment_Map::sortNodesInGrid(Cell &cell, GridDirection direction) {
    // TODO:
    // -For north/south, start a counter for x-vals (proxy for "column"). If even, loop normally over y's and add each to new list. If odd, loop over y's in reverse and add each to new list
    // -For east/west, maybe use a private variable that is sorted by y as the first value and do the same as the x sorting?
    // -For closest point, populate a list of all nodes and start at the first. as we loop over, find the closest point, add to a container, and then pop that off the list we're looping over. (use while loop over the size of this list)

    std::vector<Point> sortedPoints;

    switch (direction) {
    case GridDirection::EAST_WEST:
    {
        int column = 0;
        for(auto xIt : cell.containedNodes) {
            double xVal = xIt.first;
            // If column is odd, iterate in reverse:
            if(column % 2 == 0) {
                for(auto yIt : xIt.second) {
                    sortedPoints.push_back(Point(xVal, yIt.first, 0));
                }
            }
            else {
                for(auto yItRev = xIt.second.rbegin(); yItRev != xIt.second.rend(); ++yItRev) {
                    sortedPoints.push_back(Point(xVal, yItRev->first, 0));
                }
            }

            // Iterate column counter:
            column++;
        }
    }
        break;
    case GridDirection::NORTH_SOUTH:
    {
        int row = 0;
        for(auto yIt : cell.containedNodes_YX) {
            double yVal = yIt.first;
            // If column is odd, iterate in reverse:
            if(row % 2 == 0) {
                for(auto xIt : yIt.second) {
                    sortedPoints.push_back(Point(xIt.first, yVal, 0));
                }
            }
            else {
                for(auto xItRev = yIt.second.rbegin(); xItRev != yIt.second.rend(); ++xItRev) {
                    sortedPoints.push_back(Point(xItRev->first, yVal, 0));
                }
            }

            // Iterate column counter:
            row++;
        }
    }
        break;
    case GridDirection::CLOSEST_POINT:
    {
        std::vector<int> usedIndeces;
        for(int i = 0; i < cell.unsortedContainedPoints.size(); i++) {
            Point tmpClosestPoint(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0);
            double prevDist = std::numeric_limits<double>::max();

            int usedIndex = 0;
            if(i == 0) {
                sortedPoints.push_back(cell.unsortedContainedPoints[i]);
                usedIndeces.push_back(usedIndex);
                continue;
            }
            for(int j = 1; j < cell.unsortedContainedPoints.size(); j++) {
                // Check that we aren't comparing the same indeces or that we've already used the index.
                //      -If we have, continue
                //      -If not, check distance for closest point:
                if((i-1) == j || std::find(usedIndeces.begin(), usedIndeces.end(), j) != usedIndeces.end()){
                    continue;
                }
                else {
                    double dist = distanceBetweenPoints(sortedPoints[i-1], cell.unsortedContainedPoints[j]);
                    if(dist < prevDist) {
                        prevDist = dist;
                        tmpClosestPoint = Point(cell.unsortedContainedPoints[j]);
                        usedIndex = j;
                    }
                }
            }

            // Insert closest point:
            sortedPoints.push_back(tmpClosestPoint);

            // Insert index into vector:
            usedIndeces.push_back(usedIndex);
        }
    }
        break;
    default:
    {
        std::cout << "Invalid grid direction" << std::endl;
    }
        break;
    }

    return sortedPoints;
}

/**
 * @brief setContainedNodesYX For sorting a grid in the North/South direction, we need the nodes in Y,X order instead of X,Y order
 * @param cell Cell to set our YX pairs for
 */
void Environment_Map::setContainedNodesYX(Cell &cell) {
    double epsilon = 0.000001;
    // TODO: Set YX pairs of contained nodes:
    std::map<double, std::map<double, Node> > tmpContainedNodes_YX;
    for(auto xIt : cell.containedNodes) {
        double xVal = xIt.first;
        // If column is odd, iterate in reverse:
        std::map<double, Node> tmp_x_nodes;
        for(auto yIt : xIt.second) {

            // Find if there is a yVal in our tmpContainedNodes_YX map that corresponds to this yvalue.
            //      -If so, find if there is already an x value in that map.
            //      -If not (there shouldn't be), add it to the map
            Node node = yIt.second;
            double yVal = yIt.first;

            auto mapIt = tmpContainedNodes_YX.lower_bound(yVal);
            if(mapIt == tmpContainedNodes_YX.end()) {
                // Nothing found. Need to insert a new entry
                std::map<double, Node> tmp_x_node;
                tmp_x_node.insert(std::make_pair(xVal, node));
                tmpContainedNodes_YX.insert(std::make_pair(yVal, tmp_x_node));
            }
            else if(mapIt == tmpContainedNodes_YX.begin()) {
                // Found beginning of map. Check if closer than epsilon away.
                //      -If so, use it.
                //      -If not, insert a new entry
                if(fabs(mapIt->first - yVal) < epsilon) {
                    mapIt->second.insert(std::make_pair(xVal, node));
                }
                else {
                    std::map<double, Node> tmp_x_node;
                    tmp_x_node.insert(std::make_pair(xVal, node));
                    tmpContainedNodes_YX.insert(std::make_pair(yVal, tmp_x_node));
                }
            }
            else {
                // Check previous iterator.
                //      -If closer to the yval we are checking, then use the previous iterator
                std::map<double, std::map<double, Node> >::iterator prevIt = mapIt;
                --prevIt;
                if(fabs(prevIt->first - yVal) < fabs(mapIt->first - yVal)) {
                    prevIt->second.insert(std::make_pair(xVal, node));
                }
                else {
                    mapIt->second.insert(std::make_pair(xVal, node));
                }
            }
        }
    }

    // Set contained nodes YX:
    cell.containedNodes_YX = tmpContainedNodes_YX;
}

